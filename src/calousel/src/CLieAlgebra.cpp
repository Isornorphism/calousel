#include <calousel/CLieAlgebra.hpp>


Eigen::Matrix3d LieAlgebra_rp::skew_symmetric_matrix(const Eigen::Vector3d &v)
{
    Eigen::Matrix3d skew;
    skew << 0., -v(2), v(1),
        v(2), 0., -v(0),
        -v(1), v(0), 0.;
    return skew;
}

Eigen::Matrix3d LieAlgebra_rp::J_left(const Eigen::Vector3d& w) {
    double theta = w.norm();
    double eps = std::numeric_limits<double>::epsilon() * 100;
    Eigen::Matrix3d J;
    
    if(theta > eps) { //
        Eigen::Matrix3d skew_w = skew_symmetric_matrix(w);
        double sin_theta = sin(theta);
        double cos_theta = cos(theta);

        double term1 = (1.0 - cos_theta) / (theta * theta);
        double term2 = ((theta - sin_theta) / (theta * theta * theta));
        J = Eigen::Matrix3d::Identity() + term1 * skew_w + term2 * (skew_w*skew_w);
    }
    else {
        J = Eigen::Matrix3d::Identity();
    }
    return J;
}

Eigen::Matrix3d LieAlgebra_rp::J_left_inv(const Eigen::Vector3d& w) {
    double theta = w.norm();
    double eps = std::numeric_limits<double>::epsilon() * 100;
    Eigen::Matrix3d J_inv;
    Eigen::Matrix3d skew_w = skew_symmetric_matrix(w);

    if (theta < eps) {
        J_inv = Eigen::Matrix3d::Identity() - 0.5 * skew_w + 1.0/12.0 * (skew_w * skew_w);
    }
    else if (abs(theta - M_PI) < eps) {
        J_inv = Eigen::Matrix3d::Identity() - 0.5 * skew_w + 1.0/(M_PI*M_PI) * (skew_w * skew_w);
    }
    else {
        // Coefficients derived from the inverse of the SE(3) Jacobian J
        double sin_theta = sin(theta);
        double cos_theta = cos(theta);
        double term_coeff = 1.0 - theta * sin_theta / (2.0 * (1.0 - cos_theta));

        J_inv = Eigen::Matrix3d::Identity() - 0.5 * skew_w +
                (1.0 / (theta*theta)) * term_coeff * (skew_w * skew_w);
    }
    return J_inv;
}


Eigen::Matrix<double, 6, 6> LieAlgebra_rp::J_wt2wv(const Eigen::Vector3d& w, const Eigen::Vector3d& t) {
    /*
    return d(w, v) / d(w, t)
    d(w, v) / d(w, t) = [dw/dw dw/dt
                         dv/dw dv/dt]
                      = [I               0
                         (dJ^-1(w)/dw)t J^-1*(w)]
    
    */

    Eigen::Matrix<double, 6, 6> res = Eigen::Matrix<double, 6, 6>::Identity();
    res.block<3,3>(3,3) = LieAlgebra_rp::J_left_inv(w);

    double theta = w.norm();
    Eigen::Matrix3d skew_w = LieAlgebra_rp::skew_symmetric_matrix(w);

    std::vector<Eigen::Matrix3d> dw_hat_dw;
    dw_hat_dw.push_back((Eigen::Matrix3d() << 0, 0, 0,
                                              0, 0, -1,
                                              0, 1, 0).finished());
    dw_hat_dw.push_back((Eigen::Matrix3d() << 0, 0, 1,
                                              0, 0, 0,
                                              -1, 0, 0).finished());
    dw_hat_dw.push_back((Eigen::Matrix3d() << 0, -1, 0,
                                              1, 0, 0,
                                              0, 0, 0).finished());

    Eigen::Matrix3d common_term_1 = 1/(theta*theta) * (-2/(theta*theta) + 1/(theta*tan(theta/2)) + 1/(2*sin(theta/2)*sin(theta/2))) * (skew_w*skew_w);
    double common_term_2 = 1/(theta*theta) * (1 - theta/(2*tan(theta/2)));
    for(int i=0; i<3; i++) {
        res.block<3,1>(3,i) = (-0.5 * dw_hat_dw[i]
                            + w(i) * common_term_1
                            + common_term_2 * (dw_hat_dw[i]*skew_w + skew_w*dw_hat_dw[i])) * t;
    }
    
    return res;
}


double LieAlgebra_rp::angle_diff(se3 se3_1, se3 se3_2) {
    Eigen::Matrix3d R1 = LieAlgebra::to_SO3(se3_1.rot);
    Eigen::Matrix3d R2 = LieAlgebra::to_SO3(se3_2.rot);

    Eigen::Matrix3d R12 = R2.inverse() * R1;
    return acos((R12.trace() - double(1.0)) / double(2.0));
}

Eigen::Matrix4d LieAlgebra_rp::to_SE3(const Twist& twist) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = LieAlgebra::to_SO3(twist.w);
    T.block<3, 1>(0, 3) = J_left(twist.w) * twist.v;
    
    return T;
}


Eigen::Matrix3d LieAlgebra_rp::to_E(const Twist& twist) {
    Eigen::Matrix3d E;

    Eigen::Matrix4d T = to_SE3(twist);
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    Eigen::Vector3d t = T.block<3, 1>(0, 3);

    E.col(0) = R.col(0);
    E.col(1) = R.col(1);
    E.col(2) = t;

    return E;
}

Twist LieAlgebra_rp::to_twist(const Eigen::Matrix4d& T) {
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    Eigen::Vector3d t = T.block<3, 1>(0, 3);

    Twist twist;

    double eps = std::numeric_limits<double>::epsilon() * 100;

    if (R.isApprox(Eigen::Matrix3d::Identity(), eps)) { // No rotation
        twist.v = t;
        return twist;
    }
    else {
        twist.w = LieAlgebra::to_so3(R);
        twist.v = J_left_inv(twist.w) * t;
        return twist;
    }
}

Twist LieAlgebra_rp::to_twist(const Screw& screw) {
    return Twist(screw.w, screw.v);
}

Twist LieAlgebra_rp::to_twist(const se3& E) {
    Eigen::Matrix4d T = LieAlgebra::to_SE3(E);
    return to_twist(T);
}

Screw LieAlgebra_rp::to_screw(const Twist& twist) {
    double theta = twist.w.norm();
    double v_norm = twist.v.norm();
    double eps = std::numeric_limits<double>::epsilon() * 100;
    Screw res;

    if(theta < eps) { // pure translation
        if(v_norm < eps) {
            return res;
        }
        else {
            res.v = twist.v / v_norm;
            return res;
        }
    }
    else {
        res.w = twist.w / theta;
        res.v = twist.v / theta;
        return res;
    }
}

double LieAlgebra_rp::get_angle(const Twist& twist) {
    double theta = twist.w.norm();
    return theta > 1e-9 ? theta : 0.0;
}

void LieAlgebra_rp::renormalize_screw(Screw& screw) {
    screw.w /= screw.w.norm();
}

Eigen::Matrix4d LieAlgebra_rp::hat(const Twist& twist) {
    Eigen::Matrix4d se3_hat;
    se3_hat.block<3,3>(0,0) = skew_symmetric_matrix(twist.w);
    se3_hat.block<3,1>(0,3) = twist.v;
    return se3_hat;
}


Eigen::Matrix4d LieAlgebra_rp::hat(const Screw& screw) {
    Eigen::Matrix4d se3_hat;
    se3_hat.block<3,3>(0,0) = skew_symmetric_matrix(screw.w);
    se3_hat.block<3,1>(0,3) = screw.v;
    return se3_hat;
}

Eigen::Matrix<double, 6, 6> LieAlgebra_rp::compute_adjoint(const Eigen::Matrix4d& T) {
    Eigen::Matrix<double, 6, 6> Ad = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix3d R = T.block<3,3>(0,0);
    Eigen::Vector3d t = T.block<3,1>(0,3);

    Ad.block<3,3>(0,0) = R;
    Ad.block<3,3>(3,3) = R;
    Ad.block<3,3>(3,0) = skew_symmetric_matrix(t) * R;
    return Ad;
}

Eigen::Matrix<double, 6, 6> LieAlgebra_rp::compute_adjoint(const Twist& twist) {
    Eigen::Matrix4d T = to_SE3(twist);
    return compute_adjoint(T);
}


Eigen::Matrix<double, 6, 6> LieAlgebra_rp::compute_interframe_cov(const se3& se3_1,
                                                                  const se3& se3_2,
                                                                  const Eigen::Matrix<double, 6, 6>& cov1,
                                                                  const Eigen::Matrix<double, 6, 6>& cov2)
{
    /*
    T_rel = T2^-1 * T1
    xi_rel = xi_1 - Ad_(T1^-1 T2) * xi_2
    Cov_rel = Cov_1 + Ad_(T1^-1 T2) * Cov_2 * Ad_(T1^-1 T2)^T
    */
    Eigen::Matrix4d T1 = LieAlgebra::to_SE3(se3_1);
    Eigen::Matrix4d T2 = LieAlgebra::to_SE3(se3_2);
    Eigen::Matrix<double, 6, 6> Ad_T1invT2 = compute_adjoint(T1.inverse() * T2);
    return cov1 + Ad_T1invT2 * cov2 * Ad_T1invT2.transpose();

}
