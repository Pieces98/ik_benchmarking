#include "ik_benchmarking/ik_benchmarking.hpp"

#include <fmt/core.h>
#include <fmt/ranges.h>

#include <chrono>
#include <numeric>

#include <pose_ik/pose_ik.hpp>
#include <pro_rrt/pro_rrt.hpp>

using namespace std::chrono_literals;

void IKBenchmarking::initialize() {
    const auto seed = static_cast<unsigned int>(node_->get_parameter("random_seed").as_int());
    generator_ = std::make_unique<random_numbers::RandomNumberGenerator>(seed);

    planning_group_name_ = node_->get_parameter("planning_group").as_string();
    joint_model_group_ = robot_model_->getJointModelGroup(planning_group_name_);

    auto const &link_names = joint_model_group_->getLinkModelNames();

    if (!link_names.empty()) {
        tip_link_name_ = link_names.back();
    } else {
        RCLCPP_ERROR(logger_, "ERROR: The move group is corrupted. Links count is zero.\n");
        rclcpp::shutdown();
    }

    robot_state_->setToDefaultValues();

    if (node_->has_parameter("check_self_collision")) {
        check_self_collision_ = node_->get_parameter("check_self_collision").as_bool();
    }
    if (check_self_collision_) {
        planning_scene_.emplace(robot_model_);
        RCLCPP_INFO(logger_, "Self-collision checking is ENABLED.");
    } else {
        RCLCPP_INFO(logger_, "Self-collision checking is DISABLED (kinematics-only).");
    }
}

void IKBenchmarking::gather_data() {
    sample_size_ = static_cast<size_t>(node_->get_parameter("sample_size").as_int());
    ik_timeout_ = node_->get_parameter("ik_timeout").as_double();
    ik_iteration_display_step_ =
        static_cast<size_t>(node_->get_parameter("ik_iteration_display_step").as_int());

    const auto &joint_names = joint_model_group_->getActiveJointModelNames();
    data_file_ << "trial,found_ik,solve_time,position_error,orientation_error";
    for (const auto &name : joint_names) {
        data_file_ << ",target_" << name;
    }
    data_file_ << "\n";

    auto validation_fn = [this](const Eigen::VectorXd &solution) {
        return !check_self_collision_ ||
               pro_rrt::collisionValidationFunction(*joint_model_group_, solution, *planning_scene_);
    };

    const moveit_pro::base::LinkModel *tip_link_model = joint_model_group_->getLinkModel(tip_link_name_);
    if (tip_link_model == nullptr) {
        RCLCPP_ERROR(logger_, "Tip link '%s' is not part of planning group '%s'.",
                     tip_link_name_.c_str(), planning_group_name_.c_str());
        return;
    }

    constexpr size_t MAX_RESAMPLE_ATTEMPTS = 100;

    for (size_t i = 0; i < sample_size_; ++i) {
        if ((i + 1) % ik_iteration_display_step_ == 0) {
            RCLCPP_INFO(logger_, "Solved sample %ld/%ld ...", i + 1, sample_size_);
        }

        std::vector<double> random_joint_values;
        size_t attempts = 0;
        while (true) {
            robot_state_->setToRandomPositions(joint_model_group_, *generator_);
            robot_state_->updateLinkTransforms();

            if (!check_self_collision_) break;

            Eigen::VectorXd current(joint_model_group_->getVariableCount());
            robot_state_->copyJointGroupPositions(joint_model_group_, current);
            if (pro_rrt::collisionValidationFunction(*joint_model_group_, current, *planning_scene_)) break;

            if (++attempts >= MAX_RESAMPLE_ATTEMPTS) {
                RCLCPP_WARN(logger_,
                            "Could not sample a self-collision-free target after %zu attempts; "
                            "using last sampled state for trial %zu.",
                            MAX_RESAMPLE_ATTEMPTS, i + 1);
                break;
            }
        }

        const Eigen::Isometry3d tip_link_pose =
            robot_state_->getGlobalLinkTransform(tip_link_name_);
        robot_state_->copyJointGroupPositions(joint_model_group_, random_joint_values);

        robot_state_->setToRandomPositions(joint_model_group_, *generator_);
        robot_state_->updateLinkTransforms();
        Eigen::VectorXd seed(joint_model_group_->getVariableCount());
        robot_state_->copyJointGroupPositions(joint_model_group_, seed);

        pose_ik::PoseTarget pose_target;
        pose_target.tip_link = tip_link_model;
        pose_target.root_pose_tip = tip_link_pose;

        const auto start_time = std::chrono::high_resolution_clock::now();
        auto ik_solution = pose_ik::solveIK(
            *robot_state_, *joint_model_group_, {pose_target}, seed,
            rclcpp::Duration::from_seconds(ik_timeout_), validation_fn, pose_ik::Params());
        const auto end_time = std::chrono::high_resolution_clock::now();

        const bool found_ik = ik_solution.has_value();
        if (found_ik) {
            robot_state_->setJointGroupPositions(joint_model_group_, ik_solution.value());
            robot_state_->updateLinkTransforms();
            success_count_++;
        }

        const auto solve_time =
            std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        solve_times_.push_back(solve_time.count());

        Eigen::Isometry3d ik_tip_link_pose = robot_state_->getGlobalLinkTransform(tip_link_name_);
        Eigen::Vector3d position_diff =
            ik_tip_link_pose.translation() - tip_link_pose.translation();
        double position_error = position_diff.norm();

        Eigen::Quaterniond orientation(tip_link_pose.rotation());
        Eigen::Quaterniond ik_orientation(ik_tip_link_pose.rotation());
        double orientation_error = orientation.angularDistance(ik_orientation);

        data_file_ << std::boolalpha << i + 1 << "," << found_ik << "," << solve_time.count() << ","
                   << position_error << "," << orientation_error;
        for (const double q : random_joint_values) {
            data_file_ << "," << q;
        }
        data_file_ << "\n";
    }

    average_solve_time_ =
        std::accumulate(solve_times_.begin(), solve_times_.end(), 0.0) / solve_times_.size();
    success_rate_ = success_count_ / sample_size_;

    RCLCPP_INFO(logger_, "Success rate = %f and average IK solving time is %f microseconds\n",
                success_rate_, average_solve_time_);

    calculation_done_ = true;
}

void IKBenchmarking::run() {
    this->initialize();
    this->gather_data();

    this->data_file_.close();
}

double IKBenchmarking::get_success_rate() const { return success_rate_; }

double IKBenchmarking::get_average_solve_time() const { return average_solve_time_; }

bool IKBenchmarking::calculation_done() const { return calculation_done_; }
