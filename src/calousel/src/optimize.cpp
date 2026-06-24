#include <calousel/optimize.hpp>

using namespace calousel;

rp_calibration::rp_calibration(
    std::vector<ThreadSafeQueue<KeyFrame>>& key_frame_queues_,
    const BoardConfig& board_config,
    const std::vector<CamConfig>& cam_configs,
    int cam_num_, bool use_weight_, bool fix_reference_camera_z_to_zero_
)
    : key_frame_queues(key_frame_queues_), cam_num(cam_num_), use_weight(use_weight_), fix_reference_camera_z_to_zero(fix_reference_camera_z_to_zero_),
      board_config(board_config), cam_configs(cam_configs),
      targets(cam_num_), init_t_ac_ref(cam_num_), t_ac_ref_optimized(cam_num_), T_ac_ref_res(cam_num_)
{

}

void rp_calibration::set_discocal_target() {
    for(int c=0; c<key_frame_queues.size(); c++) {
        if(key_frame_queues[c].empty()) return;

        for(auto kf : key_frame_queues[c].snapshot()) {
            targets[c].push_back(kf.target);
        }
    }
    
    for(int j=0;j<board_config.n_y;j++){
        for(int i=0;i<board_config.n_x;i++){
            double x,y;
            x = board_config.distance*(i+1);
            y = board_config.distance*(board_config.n_y-j);
            Shape target(x,y, M_PI*board_config.r*board_config.r);
            origin_target.push_back(target);
        }
    }
}


void rp_calibration::set_keyframe_ref(int cam_ref_) {
    double min_rep_error = std::numeric_limits<double>::max();

    for(int c=0; c<key_frame_queues.size(); c++) {
        if(key_frame_queues[c].empty()) return;

        double min_rep_error_in_cycle = key_frame_queues[c][0].rep_error;
        int index_ref_in_cycle = 0;

        std::vector<double> min_rep_error_each_cycle;
        std::vector<KeyFrame> best_keyframe_each_cycle;
        std::vector<int> best_keyframe_index_each_cycle; 

        for(int i=1; i<key_frame_queues[c].size(); i++) {
            if(key_frame_queues[c][i].detection_cycle != key_frame_queues[c][i-1].detection_cycle) {
                min_rep_error_each_cycle.push_back(min_rep_error_in_cycle);
                best_keyframe_each_cycle.push_back(key_frame_queues[c][index_ref_in_cycle]);
                best_keyframe_index_each_cycle.push_back(index_ref_in_cycle);

                min_rep_error_in_cycle = std::numeric_limits<double>::max();
                index_ref_in_cycle = i;
            }
            else {
                if(key_frame_queues[c][i].rep_error < min_rep_error_in_cycle) {
                    min_rep_error_in_cycle = key_frame_queues[c][i].rep_error;
                    index_ref_in_cycle = i;
                    
                    if(cam_ref_ < 0) { // auto select
                        if(key_frame_queues[c][i].rep_error < min_rep_error) {
                        // if(key_frame_queues[c][i].cal_qual > best_cal_qual) {
                            min_rep_error = key_frame_queues[c][i].rep_error;
                            this->index_ref = std::make_tuple(c,
                                                            key_frame_queues[c][i].detection_cycle,
                                                            i);
                        }
                    }
                    else { // specific cam_ref
                        if(cam_ref_ == c) {
                            if(key_frame_queues[c][i].rep_error < min_rep_error) {
                                min_rep_error = key_frame_queues[c][i].rep_error;
                                this->index_ref = std::make_tuple(c,
                                                                key_frame_queues[c][i].detection_cycle,
                                                                i);
                            }
                        }
                    }

                }
            }
        }
        min_rep_error_each_cycle.push_back(min_rep_error_in_cycle);
        best_keyframe_each_cycle.push_back(key_frame_queues[c][index_ref_in_cycle]);
        best_keyframe_index_each_cycle.push_back(index_ref_in_cycle);


        this->keyframe_refs.push_back(best_keyframe_each_cycle);
        this->index_refs.push_back(best_keyframe_index_each_cycle);

        printf("[Cam %d] Reference KeyFrame index : ", c);
        for(auto idx : this->index_refs[c]) {
            std::cout << idx << " ";
        }
        std::cout << std::endl;
    } 
    this->keyframe_ref = key_frame_queues[std::get<0>(this->index_ref)][std::get<2>(this->index_ref)];
    this->cam_ref = std::get<0>(index_ref);
    if(cam_ref_ < 0) {
        printf("Mode : Auto select\n");
        printf("Best rep error keyframe index : Cam %d, Cycle %d, Index %d\n", std::get<0>(this->index_ref), std::get<1>(this->index_ref), std::get<2>(this->index_ref));
        printf("Reference camera : Cam %d\n", this->cam_ref);
    }
    else {
        printf("Mode : Specific cam select\n");
        printf("Reference camera : Cam %d, Index %d\n", this->cam_ref, std::get<2>(this->index_ref));
    }
}


void rp_calibration::set_init_angular_vel(const std::vector<std::vector<double>>& angular_vel_vecs) {
    int cnt = 0;
    double angular_vel_ = 0.0;
    for(int i=0; i<cam_num; i++) {
        for(auto ang_v : angular_vel_vecs[i]) {
            angular_vel_ += ang_v;
            cnt += 1;
        }
    }
    this->init_angular_vel = angular_vel_ / cnt;
    this->init_angular_vel *= M_PI/180.; // deg to rad

    std::cout << "Initial angular velocity : " << this->init_angular_vel << "rad/s"<< std::endl;
}


void rp_calibration::set_init_T_ba() {
    // 1. Initialize R_ba
    std::vector<std::pair<se3, se3>> rot_axis_vec;
    std::vector<Eigen::Vector3d> rot_so3_vec;
    for(int i=0; i<2; i++) {
        int num_keyframes = key_frame_queues[i].size();
        if(num_keyframes <= 1) {
            std::cout << "[Cam " << i << "] # of keyframes must be greater than 2." << std::endl;
            continue;
        }
        size_t start_index = 0;
        for(size_t idx=1; idx<key_frame_queues[i].size(); idx++) {
            if(key_frame_queues[i][idx].detection_cycle != key_frame_queues[i][idx-1].detection_cycle) {
                size_t frame_count = idx - start_index;
                if(frame_count > 1) {
                    rot_axis_vec.push_back({key_frame_queues[i][start_index].E,
                                            key_frame_queues[i][idx-1].E});
                }
                start_index = idx;
            }
        }
        size_t last_cycle_frame_count = key_frame_queues[i].size() - start_index;
        if(last_cycle_frame_count > 1) {
            rot_axis_vec.push_back({key_frame_queues[i][start_index].E,
                                    key_frame_queues[i].back().E});
        }
    }
    for(const auto& rot_pair : rot_axis_vec) {
        Eigen::Matrix4d start_pose = LieAlgebra::to_SE3(rot_pair.first).inverse();
        Eigen::Matrix4d end_pose = LieAlgebra::to_SE3(rot_pair.second).inverse();

        Eigen::Matrix4d delta_pose = end_pose * start_pose.inverse();
        Eigen::Vector3d rot_so3 = LieAlgebra::to_se3(delta_pose).rot;

        rot_so3 = LieAlgebra::normalize_so3(rot_so3);
        if(rot_so3.norm() < 1e-6) continue;
        rot_so3 /= rot_so3.norm();

        rot_so3_vec.push_back(rot_so3);
    }

    Eigen::Vector3d w_ba = Eigen::Vector3d::Zero();
    for(const auto& rot_so3_ : rot_so3_vec) {
        w_ba += rot_so3_;
    }
    w_ba /= w_ba.norm(); // w_ba /= rot_so3_vec.size() and w_ba /= w_ba.norm() >> w_ba /= w_ba.norm() at one time!!


    // TODO
    Eigen::Matrix3d R_ba;
    Eigen::Vector3d z_axis = w_ba;
    Eigen::Vector3d x_axis = Eigen::Vector3d::UnitX() - z_axis.dot(Eigen::Vector3d::UnitX()) * z_axis;
    x_axis /= x_axis.norm();
    Eigen::Vector3d y_axis = z_axis.cross(x_axis);
    y_axis /= y_axis.norm();
    R_ba.block<3, 1>(0, 0) = x_axis;
    R_ba.block<3, 1>(0, 1) = y_axis;
    R_ba.block<3, 1>(0, 2) = z_axis;

    std::cout << "[Initial R_ba]" << std::endl;
    std::cout << R_ba << std::endl;
    this->init_t_ba.rot = LieAlgebra::to_so3(R_ba);
    this->init_t_ba.rot = LieAlgebra::normalize_so3(this->init_t_ba.rot);
    // std::cout << "[Initial w_ba]" << std::endl;
    // std::cout << this->init_t_ba.rot / this->init_t_ba.rot.norm() << std::endl;
    // std::cout << "angle : " << this->init_t_ba.rot.norm() * 180/M_PI << "deg" << std::endl;


    // 2. Initialize t_ba : equidistance condition - use all cam, perpendicular condition - use reference cam
    std::vector<Eigen::RowVector3d> A_rows;
    std::vector<double> b_vals;

    Eigen::Vector3d t_curr;
    for(int c=0; c<key_frame_queues.size(); c++) {
        if(key_frame_queues[c].size() < 2) {
            printf("[Cam %d] # of keyframes must be greater than 2 for applying equidistance condition.\n", c);
            continue;
        }

        int curr_cycle = 0;
        Eigen::Vector3d t_ref = LieAlgebra::to_SE3(key_frame_queues[c][this->index_refs[c][curr_cycle]].E).inverse().block<3, 1>(0, 3);

        // ||t_ref - t_ba|| = ||t_curr - t_ba|| -> 2 * (t_curr - t_ref) * t_ba = ||t_curr||^2 - ||t_ref||^2
        for (size_t idx=0; idx<key_frame_queues[c].size(); idx++) {
            if(idx != 0) {
                if (key_frame_queues[c][idx].detection_cycle != key_frame_queues[c][idx-1].detection_cycle) {
                    curr_cycle = key_frame_queues[c][idx].detection_cycle;
                    t_ref = LieAlgebra::to_SE3(key_frame_queues[c][this->index_refs[c][curr_cycle]].E).inverse().block<3, 1>(0, 3);
                }
            }
            if (idx != this->index_refs[c][curr_cycle]) {
                t_curr = LieAlgebra::to_SE3(key_frame_queues[c][idx].E).inverse().block<3, 1>(0, 3);
                Eigen::Vector3d delta_tt = t_curr - t_ref;
                double norm_delta_t = delta_tt.norm();
                //normalize A row vector to 1
                A_rows.push_back((delta_tt / norm_delta_t).transpose());
                b_vals.push_back((t_curr.squaredNorm() - t_ref.squaredNorm()) / (2.0 * norm_delta_t));
            }
        }
    }

    // (t_curr - t_ba) \dot w_ba = 0
    for (size_t idx=0; idx<key_frame_queues[this->cam_ref].size(); idx++) {
        t_curr = LieAlgebra::to_SE3(key_frame_queues[this->cam_ref][idx].E).inverse().block<3, 1>(0, 3);
        A_rows.push_back(w_ba.transpose());
        b_vals.push_back(w_ba.dot(t_curr));
    }

    if (A_rows.empty()) {
        std::cout << "Error: Not enough equations could be formed." << std::endl;
        return;
    }
    
    Eigen::MatrixXd A(A_rows.size(), 3);
    Eigen::VectorXd b(b_vals.size());
    for(size_t i=0; i<A_rows.size(); i++) {
        A.row(i) = A_rows[i];
        b(i) = b_vals[i];
    }
    this->init_t_ba.trans =  A.colPivHouseholderQr().solve(b);

    // this->init_t_ba.trans = LieAlgebra::normalize_translation(this->init_t_ba.trans);
    std::cout << "[Initial t_ba]" << std::endl;
    std::cout << this->init_t_ba.trans << std::endl;
}


void rp_calibration::set_init_T_ac(int cam_id_) {
    if (cam_id_ < 0 || cam_id_ >= cam_num) return;
    const calousel::TimestampNs timestamp_ref_ns = this->keyframe_ref.timestamp_ns;
    std::vector<Sophus::SE3d> t_ac_candidates;
    for (const auto& kf : key_frame_queues[cam_id_].snapshot()) {
        Sophus::SE3d T_bc(LieAlgebra::to_SE3(kf.E).inverse());
        const double delta_t_sec = calousel::to_seconds(kf.timestamp_ns - timestamp_ref_ns);
        double angle = this->angular_vel_optimized[0] * delta_t_sec;
        Sophus::SE3d t_rot_z_SE3(Sophus::SO3d::rotZ(angle), Eigen::Vector3d::Zero());

        // T_bci = T_ba * T_rot_z * T_aci
        // T_aci = T_rot_z^-1 * T_ba^-1 * T_bci
        Sophus::SE3d t_ba_SE3(LieAlgebra::to_SE3(this->init_t_ba));
        Sophus::SE3d T_ac_candidate = t_rot_z_SE3.inverse() * t_ba_SE3.inverse() * T_bc;
        t_ac_candidates.push_back(T_ac_candidate);
    }

    Sophus::Vector6d average_se3_twist = Sophus::Vector6d::Zero();
    for (const auto& T_ac : t_ac_candidates) {
        average_se3_twist += T_ac.log();
    }
    average_se3_twist /= t_ac_candidates.size();

    Eigen::Matrix4d T_ac_ref_init = Sophus::SE3d::exp(average_se3_twist).matrix();

    se3 T_ac_ref_init_se3 = LieAlgebra::to_se3(T_ac_ref_init);


    std::cout << "[Initial T_ac" << cam_id_ << "_ref]" << std::endl;
    std::cout << T_ac_ref_init << std::endl;
    std::cout << T_ac_ref_init_se3.to_string() << std::endl;

    this->init_t_ac_ref[cam_id_] = T_ac_ref_init_se3;
}


void rp_calibration::move_initial_value_to_optimize_params() {
    this->angular_vel_optimized[0] = this->init_angular_vel;
    this->t_ba_optimized = Sophus::SE3d(LieAlgebra::to_SE3(this->init_t_ba));
    for(int cam=0; cam<cam_num; cam++) {
        this->t_ac_ref_optimized[cam] = Sophus::SE3d(LieAlgebra::to_SE3(this->init_t_ac_ref[cam]));
    }
}


double rp_calibration::optimize() {
    const calousel::TimestampNs timestamp_ref_ns = this->keyframe_ref.timestamp_ns;

    ceres::LossFunction* loss_function = new ceres::TrivialLoss;
    // ceres::LossFunction* loss_function = new ceres::HuberLoss(0.1);
    ceres::Problem problem;

    problem.AddParameterBlock(angular_vel_optimized, 1);
    problem.AddParameterBlock(t_ba_optimized.data(), Sophus::SE3d::num_parameters, new Sophus::Manifold<Sophus::SE3>());

    // Add per-camera T_ac_ref parameter blocks
    for (int cam = 0; cam < cam_num; cam++) {
        if (fix_reference_camera_z_to_zero && cam_ref == cam) {
            // t_ac_ref_optimized[cam].translation().z() = 0.0;
            auto* q_manifold = new ceres::EigenQuaternionManifold(); // (x, y, z, w)
            auto* t_manifold = new ceres::SubsetManifold(3, std::vector<int>{2}); // fix t_z
            auto* se3_tz_frozen = new ceres::ProductManifold(q_manifold, t_manifold);
            problem.AddParameterBlock(
                t_ac_ref_optimized[cam].data(),
                Sophus::SE3d::num_parameters,
                se3_tz_frozen);
        } else {
            problem.AddParameterBlock(
                t_ac_ref_optimized[cam].data(),
                Sophus::SE3d::num_parameters,
                new Sophus::Manifold<Sophus::SE3>());    
        }

    }

    // Residuals per camera
    for (int cam = 0; cam < cam_num; cam++) {
        int num_keyframes = key_frame_queues[cam].size();
        for (int i = 0; i < num_keyframes; i++) {
            ceres::CostFunction* cost_function =
                new ceres::AutoDiffCostFunction<RotationPlateFunction, 6, 1, 7, 7>(
                    new RotationPlateFunction(key_frame_queues[cam][i], timestamp_ref_ns, use_weight));
            problem.AddResidualBlock(cost_function, loss_function,
                                    angular_vel_optimized,
                                    t_ba_optimized.data(),
                                    t_ac_ref_optimized[cam].data());
        }
    }

    ceres::Solver::Options options;
    options.max_num_iterations = 200;
    options.function_tolerance = 1e-8;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    options.num_threads = 4;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << std::endl;

    // Get optimize result
    angular_vel_res = angular_vel_optimized[0];
    T_ba_res = t_ba_optimized.matrix();
    for (int cam = 0; cam < cam_num; cam++) {
        T_ac_ref_res[cam] = t_ac_ref_optimized[cam].matrix();
    }

    T_c0c_res.clear();
    theta_c0c_res.clear();
    if (cam_num >= 2) {
        for(int cam=1; cam<cam_num; cam++) {
            T_c0c_res.push_back(T_ac_ref_res[0].inverse() * T_ac_ref_res[cam]);
            theta_c0c_res.push_back(LieAlgebra_rp::get_angle(LieAlgebra_rp::to_twist(T_c0c_res.back())));
        }
    }

    return summary.final_cost;
}


double rp_calibration::optimize_reprojection_error() {
    const calousel::TimestampNs timestamp_ref_ns = this->keyframe_ref.timestamp_ns;

    ceres::LossFunction* loss_function = new ceres::TrivialLoss;
    ceres::Problem problem;


    problem.AddParameterBlock(angular_vel_optimized, 1);
    problem.AddParameterBlock(t_ba_optimized.data(), Sophus::SE3d::num_parameters, new Sophus::Manifold<Sophus::SE3>());

    for (int cam = 0; cam < cam_num; cam++) {
        if (fix_reference_camera_z_to_zero && cam_ref == cam) {
            auto* q_manifold = new ceres::EigenQuaternionManifold(); // (x, y, z, w)
            auto* t_manifold = new ceres::SubsetManifold(3, std::vector<int>{2}); // fix t_z
            auto* se3_tz_frozen = new ceres::ProductManifold(q_manifold, t_manifold);
            problem.AddParameterBlock(
                t_ac_ref_optimized[cam].data(),
                Sophus::SE3d::num_parameters,
                se3_tz_frozen);
        } else {
            problem.AddParameterBlock(
                t_ac_ref_optimized[cam].data(),
                Sophus::SE3d::num_parameters,
                new Sophus::Manifold<Sophus::SE3>());    
        }
    }
    int num_residual = origin_target.size() * 2;
    for (int cam = 0; cam < cam_num; cam++) {
        const int num_keyframes = key_frame_queues[cam].size();
        for (int i = 0; i < num_keyframes; i++) {
            ceres::CostFunction* cost_function =
                new ceres::AutoDiffCostFunction<RotationPlateFunctionReprojection, ceres::DYNAMIC, 1, 7, 7>(
                    new RotationPlateFunctionReprojection(
                        origin_target,
                        targets[cam][i],
                        board_config,
                        cam_configs[cam],
                        key_frame_queues[cam][i],
                        timestamp_ref_ns),
                    num_residual);

            problem.AddResidualBlock(cost_function, loss_function,
                                     angular_vel_optimized,
                                     t_ba_optimized.data(),
                                     t_ac_ref_optimized[cam].data());
        }
    }


    // --- Optimize ---
    ceres::Solver::Options options;
    options.max_num_iterations = 200;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.num_threads = 4; // modified

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << std::endl;

    angular_vel_res = angular_vel_optimized[0];
    T_ba_res = t_ba_optimized.matrix();
    for (int cam = 0; cam < cam_num; cam++) {
        T_ac_ref_res[cam] = t_ac_ref_optimized[cam].matrix();
    }

    if (cam_num >= 2) {
        for(int cam=1; cam<cam_num; cam++) {
            T_c0c_res.push_back(T_ac_ref_res[0].inverse() * T_ac_ref_res[cam]);
            theta_c0c_res.push_back(LieAlgebra_rp::get_angle(LieAlgebra_rp::to_twist(T_c0c_res.back())));
        }
    }

    return summary.final_cost;
}

Circle3D rp_calibration::fitCircleTo3DPoints(const std::vector<Eigen::Vector3d>& points) {
    if (points.size() < 3) {
        throw std::runtime_error("At least 3 points are required to fit a circle.");
    }

    Eigen::Vector3d centroid(0, 0, 0);
    for (const auto& p : points) {
        centroid += p;
    }
    centroid /= points.size();
    Eigen::MatrixXd A(points.size(), 3);
    for (size_t i = 0; i < points.size(); ++i) {
        A.row(i) = points[i] - centroid;
    }
    Eigen::Matrix3d cov = A.transpose() * A / (points.size() - 1);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(cov);
    Eigen::Vector3d plane_normal = eigen_solver.eigenvectors().col(0);
    if (plane_normal.y() < 0) {
        plane_normal = -plane_normal;
    }

    Eigen::Vector3d u = eigen_solver.eigenvectors().col(1);
    Eigen::Vector3d v = eigen_solver.eigenvectors().col(2);
    std::vector<Eigen::Vector2d> points_2d;
    for (const auto& p : points) {
        points_2d.emplace_back(u.dot(p - centroid), v.dot(p - centroid));
    }

    Eigen::MatrixXd B(points_2d.size(), 3);
    Eigen::VectorXd d(points_2d.size());
    for (size_t i = 0; i < points_2d.size(); ++i) {
        double x = points_2d[i].x();
        double y = points_2d[i].y();
        B(i, 0) = 2.0 * x;
        B(i, 1) = 2.0 * y;
        B(i, 2) = 1.0;
        d(i) = x * x + y * y;
    }
    Eigen::Vector3d c = B.colPivHouseholderQr().solve(d);
    double center_x_2d = c[0];
    double center_y_2d = c[1];
    double radius = std::sqrt(c[2] + center_x_2d * center_x_2d + center_y_2d * center_y_2d);
    Eigen::Vector3d center_3d = centroid + center_x_2d * u + center_y_2d * v;

    return {center_3d, plane_normal, radius};
}


double rp_calibration::calculateRmsErrorWithCircle3D(const std::vector<Eigen::Vector3d>& points, const Circle3D& circle) {
    if (points.empty()) {
        return 0.0;
    }

    double sum_of_squared_errors = 0.0;

    for (const auto& point : points) {
        Eigen::Vector3d vec_to_center = point - circle.center;
        Eigen::Vector3d projected_vec = vec_to_center - vec_to_center.dot(circle.normal) * circle.normal;
        Eigen::Vector3d closest_point_on_circle = circle.center + circle.radius * projected_vec.normalized();
        sum_of_squared_errors += (point - closest_point_on_circle).squaredNorm();
    }

    return std::sqrt(sum_of_squared_errors / points.size());    
}

void rp_calibration::calculate_rep_error() {
    const calousel::TimestampNs timestamp_ref_ns = this->keyframe_ref.timestamp_ns;
    MomentsTracker tracker(board_config.n_d);

    double total_sq = 0.0;
    int total_kf = 0;

    if (static_cast<int>(rep_error_cam.size()) != cam_num) {
        rep_error_cam.assign(cam_num, 0.0);
    }

    std::cout << "--------------------- Reprojection error -----------------------" << std::endl;

    for (int cam = 0; cam < cam_num; cam++) {
        const int num_keyframes = key_frame_queues[cam].size();
        if (num_keyframes == 0) {
            rep_error_cam[cam] = 0.0;
            continue;
        }

        double cam_sq = 0.0;
        Params params = cam_configs[cam].intrinsic;
        for (int i = 0; i < num_keyframes; i++) {
            const double delta_t_sec = calousel::to_seconds(key_frame_queues[cam][i].timestamp_ns - timestamp_ref_ns);
            const double angle = this->angular_vel_res * delta_t_sec;

            Eigen::Matrix3d rot_z = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).matrix();
            Eigen::Matrix4d rot_z_SE3 = Eigen::Matrix4d::Identity();
            rot_z_SE3.block<3, 3>(0, 0) = rot_z;

            const Eigen::Matrix4d predicted_T_BC = this->T_ba_res * rot_z_SE3 * this->T_ac_ref_res[cam];
            Eigen::Matrix4d predicted_T_CB = predicted_T_BC.inverse();
            const Eigen::Matrix3d E = LieAlgebra_rp::to_E(predicted_T_CB);

            for (size_t idx = 0; idx < origin_target.size(); idx++) {
                const double wx = origin_target[idx].x;
                const double wy = origin_target[idx].y;

                Point p_i = tracker.project(wx, wy, board_config.r, params, E, 0);
                const double u_e = p_i.x;
                const double v_e = p_i.y;

                const double u_o = targets[cam][i][idx].x;
                const double v_o = targets[cam][i][idx].y;

                cam_sq += (u_e - u_o) * (u_e - u_o) + (v_e - v_o) * (v_e - v_o);
            }
        }

        rep_error_cam[cam] = std::sqrt(cam_sq / (num_keyframes * origin_target.size()));
        std::cout << "Reprojection error of cam" << cam << " : " << rep_error_cam[cam] << std::endl;

        total_sq += cam_sq;
        total_kf += num_keyframes;
    }

    if (total_kf > 0) {
        rep_error = std::sqrt(total_sq / (total_kf * origin_target.size()));
    } else {
        rep_error = 0.0;
    }

    std::cout << "Total reprojection error : " << rep_error << std::endl;
    std::cout << "----------------------------------------------------------------" << std::endl;
}





bool rp_calibration::save_extrinsic_calibration_result(const std::string& result_path, const calousel::Logger& logger) {
    namespace fs = std::filesystem;

    try {
        fs::path base_dir(result_path);
        if (!fs::exists(base_dir)) {
            fs::create_directories(base_dir);
            if (logger.info) logger.info(std::string("Created result directory: ") + base_dir.string());
        }
    }
    catch (const fs::filesystem_error& e) {
        if (logger.error) logger.error(std::string("Failed to create result directory ") + result_path + ": " + e.what());
        return false;
    }

    fs::path yaml_file_path = fs::path(result_path) / "extrinsic_calibration_data.yaml";
    YAML::Emitter out;
    out.SetDoublePrecision(6); 
    
    out << YAML::BeginMap;
    out << YAML::Key << "extrinsic_calibration";

    out << YAML::Value << YAML::BeginMap;

    // N-camera output
    for (int cam = 0; cam < cam_num; cam++) {
        std::string T_name = "T_AC" + std::to_string(cam) + "_ref";
        out << YAML::Key << T_name << YAML::Value << YAML::convert<Eigen::Matrix4d>::encode(this->T_ac_ref_res[cam]);
    }
    // (N-1)-camera output
    for (int cam = 0; cam < cam_num-1; cam++) {
        std::string T_name = "T_C0C" + std::to_string(cam+1) + "_ref";
        out << YAML::Key << T_name << YAML::Value << YAML::convert<Eigen::Matrix4d>::encode(this->T_c0c_res[cam]);
    }
    for (int cam = 0; cam < cam_num-1; cam++) {
        std::string theta_name = "theta_C0C" + std::to_string(cam+1) + "_ref";
        out << YAML::Key << theta_name << YAML::Value << this->theta_c0c_res[cam];
    }

    out << YAML::EndMap;


    out << YAML::Key << "kinematic_parameters" << YAML::Value << YAML::BeginMap;

    out << YAML::Key << "T_BA" << YAML::Value << YAML::convert<Eigen::Matrix4d>::encode(T_ba_res);
    out << YAML::Key << "w_BA" << YAML::Value <<  YAML::convert<Eigen::Vector3d>::encode(T_ba_res.block<3, 1>(0, 2));
    out << YAML::Key << "t_BA" << YAML::Value <<  YAML::convert<Eigen::Vector3d>::encode(T_ba_res.block<3, 1>(0, 3));
    out << YAML::Key << "Omega" << YAML::Value << angular_vel_res;
    out << YAML::EndMap;


    out << YAML::Key << "optimize_parameters" << YAML::Value << YAML::BeginMap;

    out << YAML::Key << "rep_error_cam" << YAML::Value << YAML::BeginSeq;
    for (double v : this->rep_error_cam) out << v;
    out << YAML::EndSeq;
    out << YAML::Key << "rep_error" << YAML::Value << this->rep_error;


    out << YAML::EndMap;

    out << YAML::EndMap;

    try {
        std::ofstream fout(yaml_file_path.string());
        fout << out.c_str();
        fout.close();
        if (logger.info) logger.info(std::string("Successfully saved extrinsic calibration result to YAML: ") + yaml_file_path.string());
        return true;
    }
    catch (const std::exception& e) {
        if (logger.error) logger.error(std::string("Failed to save YAML file ") + yaml_file_path.string() + ": " + e.what());
        return false;
    }
}