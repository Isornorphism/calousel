#ifndef CALOUSEL__CLIEALGEBRA_HPP_
#define CALOUSEL__CLIEALGEBRA_HPP_


#include <CLieAlgebra.h>
#include "ceres/jet.h"

template <typename T>
struct Screw_T {
    Eigen::Matrix<T, 3, 1> w; // normalize to 1
    Eigen::Matrix<T, 3, 1> v;
    Screw_T(const Eigen::Matrix<T, 3, 1>& w_, const Eigen::Matrix<T, 3, 1>& v_) : w(w_), v(v_){};
    //Screw(const Eigen::Matrix<T, 6, 1>& xi_) : w(xi_.block<3, 1>(0,0)), v(xi_.block<3,1>(3,0)){};
    Screw_T(const T* xi_ptr) {
        Eigen::Map<const Eigen::Matrix<T, 6, 1>> xi_map(xi_ptr);
        w = xi_map.template head<3>();
        v = xi_map.template tail<3>();
    }
    Screw_T(const T* w_ptr, const T* v_ptr) {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> xi_w_map(w_ptr);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> xi_v_map(v_ptr);
        w = xi_w_map;
        v = xi_v_map;
    }
    Screw_T() : w(Eigen::Matrix<T, 3, 1>::Zero()), v(Eigen::Matrix<T, 3, 1>::Zero()){};
    std::string to_string() const {
        std::string message = std::to_string(w(0))+"\t"+std::to_string(w(1))+"\t"+std::to_string(w(2))+
        "\t"+std::to_string(v(0))+"\t"+std::to_string(v(1))+"\t"+std::to_string(v(2));
        return message;
    }
};

typedef Screw_T<double> Screw;

template <typename T>
struct Twist_T {
    Eigen::Matrix<T, 3, 1> w;
    Eigen::Matrix<T, 3, 1> v;
    Twist_T(const Eigen::Matrix<T, 3, 1>& w_, const Eigen::Matrix<T, 3, 1>& v_) : w(w_), v(v_){};
    //Screw(const Eigen::Matrix<T, 6, 1>& xi_) : w(xi_.block<3, 1>(0,0)), v(xi_.block<3,1>(3,0)){};
    Twist_T(const T* xi_ptr) {
        Eigen::Map<const Eigen::Matrix<T, 6, 1>> xi_map(xi_ptr);
        
        w = xi_map.template head<3>();
        v = xi_map.template tail<3>();
    }
    Twist_T<T> operator*(T scalar) const {
        return Twist_T<T>(w*scalar, v*scalar);
    }
    Twist_T() : w(Eigen::Matrix<T, 3, 1>::Zero()), v(Eigen::Matrix<T, 3, 1>::Zero()){};
    std::string to_string() const {
        std::string message = std::to_string(w(0))+"\t"+std::to_string(w(1))+"\t"+std::to_string(w(2))+
        "\t"+std::to_string(v(0))+"\t"+std::to_string(v(1))+"\t"+std::to_string(v(2));
        return message;
    }
};

typedef Twist_T<double> Twist;

using namespace std;

class LieAlgebra_rp{
public:
    static Eigen::Matrix3d skew_symmetric_matrix(const Eigen::Vector3d& v);
    static Eigen::Matrix3d J_left(const Eigen::Vector3d& w);
    static Eigen::Matrix3d J_left_inv(const Eigen::Vector3d& w);

    static double angle_diff(se3 se3_1, se3 se3_2);
    //template<typename T>
    static Eigen::Matrix4d to_SE3(const Twist& twist);

    static Eigen::Matrix3d to_E(const Twist& twist);
    // template<typename T>
    static Twist to_twist(const Eigen::Matrix4d& T);

    // template<typename T>
    static Twist to_twist(const Screw& screw);

    static Twist to_twist(const se3& E);

    // template<typename T>
    // static Twist to_twist(const se3_T<T>& E);
    // template<typename T>
    static Screw to_screw(const Twist& twist);

    static double get_angle(const Twist& twist);

    static void renormalize_screw(Screw& screw);

    static Eigen::Matrix4d hat(const Twist& twist);
    static Eigen::Matrix4d hat(const Screw& screw);

    static Eigen::Matrix<double, 6, 6> compute_adjoint(const Eigen::Matrix4d& T);
    static Eigen::Matrix<double, 6, 6> compute_adjoint(const Twist& twist);
    
    static Eigen::Matrix<double, 6, 6> compute_interframe_cov(const se3& se3_1,
                                                                const se3& se3_2,
                                                                const Eigen::Matrix<double, 6, 6>& cov1,
                                                                const Eigen::Matrix<double, 6, 6>& cov2);
    
    static Eigen::Matrix<double, 6, 6> J_wt2wv(const Eigen::Vector3d& w, const Eigen::Vector3d& v);

    // template ftn
    template<typename T>
    static Eigen::Matrix<T, 3, 3> skew_symmetric_matrix(const Eigen::Matrix<T, 3, 1>& v) {
        Eigen::Matrix<T, 3, 3> skew;
        skew << T(0.), -v(2), v(1),
                v(2), T(0.), -v(0),
                -v(1), v(0), T(0.);
        return skew;
    }

    template<typename T>
    static Eigen::Matrix<T, 3, 3> J_left(const Eigen::Matrix<T, 3, 1>& w) {
        T theta = w.norm();
        T eps = T(std::numeric_limits<double>::epsilon() * 100);
        Eigen::Matrix<T, 3, 3> J;
        
        if(theta > eps) { //
            Eigen::Matrix<T, 3, 3> skew_w = skew_symmetric_matrix(w);
            T sin_theta = ceres::sin(theta);
            T cos_theta = ceres::cos(theta);

            T term1 = ((T(1.0) - cos_theta) / (theta * theta));
            T term2 = ((theta - sin_theta) / (theta * theta * theta));
            J = Eigen::Matrix<T, 3, 3>::Identity() + term1 * skew_w + term2 * (skew_w*skew_w);
        }
        else {
            J = Eigen::Matrix<T, 3, 3>::Identity();
        }
        return J;
    }

    template<typename T>
    static Eigen::Matrix<T, 3, 3> J_left_inv(const Eigen::Matrix<T, 3, 1>& w) {
        T theta = w.norm();
        T eps = T(std::numeric_limits<double>::epsilon() * 100);
        Eigen::Matrix<T, 3, 3> J_inv;
        Eigen::Matrix<T, 3, 3> skew_w = skew_symmetric_matrix(w);

        if (theta < eps) {
            J_inv = Eigen::Matrix<T, 3, 3>::Identity() - T(0.5) * skew_w + T(1.0/12.0) * (skew_w * skew_w);
        }
        else if (ceres::abs(theta - T(M_PI)) < eps) {
            J_inv = Eigen::Matrix<T, 3, 3>::Identity() - T(0.5) * skew_w + T(1.0/(M_PI*M_PI)) * (skew_w * skew_w);
        }
        else {
            // Coefficients derived from the inverse of the SE(3) Jacobian J
            T sin_theta = ceres::sin(theta);
            T cos_theta = ceres::cos(theta);
            T term_coeff = (T(1.0) - theta * sin_theta / (T(2.0) * (T(1.0) - cos_theta)));

            J_inv = Eigen::Matrix<T, 3, 3>::Identity() - T(0.5) * skew_w +
                    (T(1.0) / (theta*theta)) * term_coeff * (skew_w * skew_w);
        }

        return J_inv;
    }

    template<typename T>
    static Eigen::Matrix<T, 4, 4> to_SE3_T(const Twist_T<T>& twist) {
        Eigen::Matrix<T, 4, 4> SE3 = Eigen::Matrix<T, 4, 4>::Identity();
        SE3.template block<3, 3>(0, 0) = LieAlgebra::to_SO3(twist.w);
        SE3.template block<3, 1>(0, 3) = J_left(twist.w) * twist.v;
        
        return SE3;
    }

    template<typename T>
    static Eigen::Matrix<T, 6, 1> to_twist_vec_T(const Eigen::Matrix<T, 4, 4>& SE3) {
        Eigen::Matrix<T, 3, 3> R = SE3.template block<3, 3>(0, 0);
        Eigen::Matrix<T, 3, 1> t = SE3.template block<3, 1>(0, 3);

        Eigen::Matrix<T, 6, 1> twist_vec;

        T eps = T(std::numeric_limits<double>::epsilon() * 100);

        if (R.isApprox(Eigen::Matrix<T, 3, 3>::Identity(), eps)) { // No rotation
            twist_vec.template block<3,1>(3,0) = t;
            return twist_vec;
        }
        else {
            // std::cout << "test1" << std::endl;
            Eigen::Matrix<T, 3, 1> w = LieAlgebra::to_so3(R);
            // std::cout << "test2" << std::endl;
            twist_vec.template block<3,1>(0,0) = w;
            Eigen::Matrix<T, 3, 1> v = J_left_inv(w) * t;
            twist_vec.template block<3,1>(3,0) = v;
            return twist_vec;
        }
    }

    template<typename T>
    static Twist_T<T> to_twist_T(const Screw_T<T>& screw) {
        return Twist_T<T>(screw.w, screw.v);
    }

    template<typename T>
    static void renormalize_screw(Screw_T<T>& screw) {
        screw.w /= screw.w.norm();
    }

    template<typename T>
    static Eigen::Matrix<T, 3, 3> to_E(Eigen::Matrix<T, 4, 4>& SE3) {
        Eigen::Matrix<T, 3, 3> Rot, E;
        Rot = SE3.template block<3, 3>(0, 0);
        E.col(0)= Rot.col(0);
        E.col(1)= Rot.col(1);
        E.col(2)= SE3.template block<3, 1> (0, 3);

        return E;
    }


private:
 
};

#endif