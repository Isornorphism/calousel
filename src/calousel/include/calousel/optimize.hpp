#ifndef CALOUSEL__OPTIMIZE_HPP_
#define CALOUSEL__OPTIMIZE_HPP_

#include <memory>
#include <chrono>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include "ceres/ceres.h"
#include "ceres/manifold.h"
#include "ceres/product_manifold.h"
#include <Eigen/Eigenvalues>
#include <sophus/se3.hpp>
#include <sophus/ceres_manifold.hpp>
#include <sophus/ceres_typetraits.hpp>

#include <calousel/CLieAlgebra.hpp>
#include <calousel/utils.hpp>
#include <calousel/logging.hpp>
#include <CMomentsTracker.h>

namespace calousel
{
class rp_calibration
{
public:
    rp_calibration(std::vector<ThreadSafeQueue<KeyFrame>>& key_frame_queues_,
                   const BoardConfig& board_config,
                   const std::vector<CamConfig>& cam_configs,
                   int cam_num_, bool use_weight_, bool fix_reference_camera_z_to_zero_);
    ~rp_calibration(){};
    
    
    void set_discocal_target();
    void set_keyframe_ref(int cam_ref_=-1); // -1 : auto select
    void set_init_angular_vel(const std::vector<std::vector<double>>& angular_vel_vecs);

    void set_init_T_ba(); // use coplanar, equidistance condition
    void set_init_T_ac(int cam_id_);
    void move_initial_value_to_optimize_params();
    
    double optimize();
    double optimize_reprojection_error();

    void calculate_rep_error();
    bool save_extrinsic_calibration_result(const std::string& result_path, const calousel::Logger& logger);

    Circle3D fitCircleTo3DPoints(const std::vector<Eigen::Vector3d>& points);
    double calculateRmsErrorWithCircle3D(const std::vector<Eigen::Vector3d>& points, const Circle3D& circle);

private:
    std::vector<ThreadSafeQueue<KeyFrame>>& key_frame_queues;
    int cam_num;

    // Reference keyframe for each detection cycle
    std::vector<std::vector<KeyFrame>> keyframe_refs; //[cam][cycle]
    std::vector<std::vector<int>> index_refs;
    // Reference keyframe : best cal_qual of all keyframe
    KeyFrame keyframe_ref;
    std::tuple<int, int, int> index_ref; // c0/c1, cycle, index
    int cam_ref; // c0/c1

    // discocal parameter
    BoardConfig board_config;
    std::vector<CamConfig> cam_configs;
    std::vector<std::vector<std::vector<Shape>>> targets; //[cam][kf][target_idx]
    std::vector<Shape> origin_target;

    // Initial value
    double init_angular_vel;
    se3 init_t_ba;
    std::vector<se3> init_t_ac_ref;
    
    bool use_weight, fix_reference_camera_z_to_zero;

    // Optimize value
    double angular_vel_optimized[1];
    Sophus::SE3d t_ba_optimized;
    std::vector<Sophus::SE3d> t_ac_ref_optimized;

    // Result value
    Eigen::Matrix4d T_ba_res;
    std::vector<Eigen::Matrix4d> T_ac_ref_res;
    std::vector<Eigen::Matrix4d> T_c0c_res;
    std::vector<double> theta_c0c_res;
    double angular_vel_res;

    std::vector<double> rep_error_cam;
    double rep_error = 0.0;
};


inline Sophus::SE3d discocal_pose_to_sophus(const se3& pose) {
    return Sophus::SE3d(LieAlgebra::to_SE3(pose));
}

inline Eigen::Matrix<double, 6, 6> sqrt_information_from_covariance(
    const Eigen::Matrix<double, 6, 6>& cov_in) {
    Eigen::Matrix<double, 6, 6> cov = 0.5 * (cov_in + cov_in.transpose());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(cov);
    if (solver.info() != Eigen::Success) {
        return Eigen::Matrix<double, 6, 6>::Identity();
    }

    Eigen::Matrix<double, 6, 1> eig = solver.eigenvalues();
    const double max_eig = eig.maxCoeff();
    const double eig_floor = std::max(1e-12, max_eig * 1e-12);
    for (int i = 0; i < 6; i++) {
        if (!std::isfinite(eig(i)) || eig(i) < eig_floor) eig(i) = eig_floor;
    }

    const Eigen::Matrix<double, 6, 6> information =
        solver.eigenvectors() * eig.cwiseInverse().asDiagonal()
        * solver.eigenvectors().transpose();

    Eigen::Matrix<double, 6, 6> sqrt_information =
        Eigen::Matrix<double, 6, 6>::Zero();

    // Empirically, we can take the square root of the diagonal elements of the information matrix to get the square root information matrix.
    for (int i = 0; i < 6; i++) {
        const double info_ii = information(i, i);
        if (std::isfinite(info_ii) && info_ii > 0.0) {
            sqrt_information(i, i) = std::sqrt(info_ii);
        }
    }
    return sqrt_information;
}

inline Eigen::Matrix<double, 6, 6> discocal_covariance_to_tcb_log_covariance(
    const se3& t_cb_obs,
    const Eigen::Matrix<double, 6, 6>& cov_wt_tcb) {
    using Vector6d = Eigen::Matrix<double, 6, 1>;
    const Sophus::SE3d T_cb_obs = discocal_pose_to_sophus(t_cb_obs);
    const Sophus::SE3d T_cb_obs_inv = T_cb_obs.inverse();

    Vector6d x0;
    x0.template head<3>() = t_cb_obs.rot;
    x0.template tail<3>() = t_cb_obs.trans;

    auto residual_from_discocal_param = [&](const Vector6d& x) -> Vector6d {
        se3 perturbed;
        perturbed.rot = x.template head<3>();
        perturbed.trans = x.template tail<3>();
        return (discocal_pose_to_sophus(perturbed) * T_cb_obs_inv).log();
    };

    Eigen::Matrix<double, 6, 6> J;
    for (int i = 0; i < 6; i++) {
        const double eps = 1e-7;
        Vector6d xp = x0;
        Vector6d xm = x0;
        xp(i) += eps;
        xm(i) -= eps;
        J.col(i) = (residual_from_discocal_param(xp) - residual_from_discocal_param(xm)) / (2.0 * eps);
    }

    Eigen::Matrix<double, 6, 6> cov_residual = J * cov_wt_tcb * J.transpose();
    return 0.5 * (cov_residual + cov_residual.transpose());
}

struct RotationPlateFunction {
    RotationPlateFunction(const KeyFrame& keyframe_, calousel::TimestampNs timestamp_ref_ns_, bool use_weight_)
        : keyframe(keyframe_),
          timestamp_ref_ns(timestamp_ref_ns_),
          use_weight(use_weight_),
          sqrt_information_twist(Eigen::Matrix<double, 6, 6>::Identity()) {
        if (use_weight) {
            const Eigen::Matrix<double, 6, 6> cov_tcb_log =
                discocal_covariance_to_tcb_log_covariance(keyframe.E, keyframe.se3_cov);
            sqrt_information_twist = sqrt_information_from_covariance(cov_tcb_log);
        }
    }

    template <typename T>
    bool operator()(
        const T* const omega,
        const T* const t_ba,
        const T* const t_ac_ref,
        T* residuals_raw) const {

        const T Omega = omega[0];
        const T delta_t_sec = T(calousel::to_seconds(keyframe.timestamp_ns - timestamp_ref_ns));
        const T angle = Omega * delta_t_sec;
        Sophus::SO3<T> rot_z = Sophus::SO3<T>::rotZ(angle);
        Sophus::SE3<T> t_rot_z_SE3(rot_z, Eigen::Vector3<T>::Zero());

        Eigen::Map<const Sophus::SE3<T>> t_ba_SE3(t_ba);
        Eigen::Map<const Sophus::SE3<T>> t_ac_ref_SE3(t_ac_ref);

        const Sophus::SE3<T> predicted_T_BC = t_ba_SE3 * t_rot_z_SE3 * t_ac_ref_SE3;
        const Sophus::SE3<T> predicted_T_CB = predicted_T_BC.inverse();
        const Sophus::SE3<T> measured_T_CB = discocal_pose_to_sophus(keyframe.E).template cast<T>();

        const Eigen::Matrix<T, 6, 1> twist =
            (measured_T_CB * predicted_T_CB.inverse()).log();

        if (use_weight) {
            Eigen::Map<Eigen::Matrix<T, 6, 1>> weighted_residuals(residuals_raw);
            weighted_residuals = sqrt_information_twist.template cast<T>() * twist;
        } else {
            const T trans_scale = T(10.0);
            const T rot_scale = T(1.0);

            residuals_raw[0] = twist[0] * trans_scale;
            residuals_raw[1] = twist[1] * trans_scale;
            residuals_raw[2] = twist[2] * trans_scale;

            residuals_raw[3] = twist[3] * rot_scale;
            residuals_raw[4] = twist[4] * rot_scale;
            residuals_raw[5] = twist[5] * rot_scale;
        }
        return true;
    }

private:
    KeyFrame keyframe;
    calousel::TimestampNs timestamp_ref_ns;
    bool use_weight;
    Eigen::Matrix<double, 6, 6> sqrt_information_twist;
};


struct RotationPlateFunctionReprojection {
    RotationPlateFunctionReprojection(
        const std::vector<Shape>& origin_target_,
        const std::vector<Shape>& target_,
        const BoardConfig& board_config_,
        const CamConfig& cam_config_,
        const KeyFrame& keyframe_,
        calousel::TimestampNs timestamp_ref_ns_
    )
        : origin_target(origin_target_), target(target_), board_config(board_config_), cam_config(cam_config_), keyframe(keyframe_), timestamp_ref_ns(timestamp_ref_ns_)
    {   
        sqrt_L_matrices_.resize(target.size());

        for (size_t i = 0; i < target.size(); ++i) {
            Eigen::Matrix2d cov;
            cov << target[i].Kxx, target[i].Kxy, 
                   target[i].Kxy, target[i].Kyy;
            
            Eigen::LLT<Eigen::Matrix2d> llt(cov.inverse());
            sqrt_L_matrices_[i] = llt.matrixL().transpose();
        }
    }

    template <typename T>
    bool operator()(
        const T* const omega,
        const T* const t_ba,
        const T* const t_ac_ref,
        T* residual) const {
        
        MomentsTracker tracker(board_config.n_d);

        const Params& p = cam_config.intrinsic;
        Params_T<T> params(
            static_cast<T>(p.fx),
            static_cast<T>(p.fy),
            static_cast<T>(p.cx),
            static_cast<T>(p.cy),
            static_cast<T>(p.skew),
            static_cast<T>(p.d[0]),
            static_cast<T>(p.d[1]),
            static_cast<T>(p.d[2]),
            static_cast<T>(p.d[3])
        );

        T Omega = omega[0];
        const T delta_t_sec = T(calousel::to_seconds(keyframe.timestamp_ns - timestamp_ref_ns));
        T angle = Omega * delta_t_sec;
        Sophus::SO3<T> rot_z = Sophus::SO3<T>::rotZ(angle);
        Sophus::SE3<T> t_rot_z_SE3(rot_z, Eigen::Vector3<T>::Zero());
        
        Eigen::Map<const Sophus::SE3<T>> t_ba_SE3(t_ba);
        
        Eigen::Map<const Sophus::SE3<T>> t_ac_ref_SE3(t_ac_ref);

        // Predict pose
        Sophus::SE3<T> predicted_T_BC = t_ba_SE3 * t_rot_z_SE3 * t_ac_ref_SE3;
        Eigen::Matrix<T, 4, 4> predicted_T_CB = predicted_T_BC.inverse().matrix();
        Eigen::Matrix<T, 3, 3> E = LieAlgebra_rp::to_E(predicted_T_CB);

        for(size_t i=0; i<origin_target.size(); i++) {
            T wx = static_cast<T>(origin_target[i].x);
            T wy = static_cast<T>(origin_target[i].y);

            Point_T<T> p_i = tracker.project<T>(wx, wy, static_cast<T>(board_config.r), params, E, 0);
            T u_e = p_i.x;
            T v_e = p_i.y;

            T u_o = static_cast<T>(target[i].x);
            T v_o = static_cast<T>(target[i].y);


            Eigen::Matrix<T, 2, 2> L = sqrt_L_matrices_[i].template cast<T>();
            
            Eigen::Matrix<T, 2, 1> error_vec;
            error_vec << u_o - u_e, v_o - v_e;

            Eigen::Matrix<T, 2, 1> weighted_error = L * error_vec;
            
            residual[2*i]     = weighted_error[0];
            residual[2*i + 1] = weighted_error[1];
        }

        return true;
    }

private:
    KeyFrame keyframe;
    calousel::TimestampNs timestamp_ref_ns;
    BoardConfig board_config;
    CamConfig cam_config;
    std::vector<Shape> origin_target;
    std::vector<Shape> target;
    // MomentsTracker* tracker;
    std::vector<Eigen::Matrix2d> sqrt_L_matrices_;
};





struct PriorCostFunctor {
    PriorCostFunctor(double initial_value, double std_dev, int index)
        : initial_value_(initial_value), std_dev_(std_dev), index_(index) {}

    template <typename T>
    bool operator()(const T* const parameter_block, T* residual) const {
        const T& parameter = parameter_block[index_];
        residual[0] = (parameter - T(initial_value_)) / T(std_dev_);
        return true;
    }
private:
    const double initial_value_;
    const double std_dev_;
    const int index_;
};


}
#endif
