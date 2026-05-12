#pragma once

#include <limits.h>
#include <moveit_pro_base/planning_scene/planning_scene.hpp>
#include <moveit_pro_base/robot_model_loader/robot_model_loader.hpp>
#include <random_numbers/random_numbers.h>

#include <fstream>
#include <optional>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief IK benchmarking driver for the MoveIt Pro environment.
 *
 * Loads the robot model via MoveIt Pro's RobotModelLoader (reading
 * robot_description / robot_description_semantic from the node's parameters,
 * which the launch file populates from disk) and runs random-sample IK trials
 * through pose_ik::solveIK, writing per-trial timing and error metrics to CSV.
 */
class IKBenchmarking {
   public:
    explicit IKBenchmarking(rclcpp::Node::SharedPtr node)
        : node_(node),
          logger_(node->get_logger()),
          robot_model_loader_(node),
          robot_model_(robot_model_loader_.getModel()),
          robot_state_(new moveit_pro::base::RobotState(robot_model_)),
          calculation_done_(false) {
        data_file_.open("ik_benchmarking_data.csv", std::ios::app);
    }

    IKBenchmarking(const std::string& node_name,
                   const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : node_(rclcpp::Node::make_shared(node_name, options)),
          logger_(node_->get_logger()),
          robot_model_loader_(node_),
          robot_model_(robot_model_loader_.getModel()),
          robot_state_(new moveit_pro::base::RobotState(robot_model_)),
          calculation_done_(false) {
        data_file_.open("ik_benchmarking_data.csv", std::ios::app);
    }

    IKBenchmarking(const std::string& node_name, const std::string& solver,
                   const std::string output_file,
                   const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : node_(rclcpp::Node::make_shared(node_name, options)),
          logger_(node_->get_logger()),
          robot_model_loader_(node_),
          robot_model_(robot_model_loader_.getModel()),
          robot_state_(new moveit_pro::base::RobotState(robot_model_)),
          calculation_done_(false) {
        data_file_.open(std::string(solver) + "_" + output_file + ".csv", std::ios::app);
    }

    void run();
    double get_success_rate() const;
    double get_average_solve_time() const;
    bool calculation_done() const;

   private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Logger logger_;

    moveit_pro::base::robot_model_loader::RobotModelLoader robot_model_loader_;
    moveit_pro::base::RobotModelPtr robot_model_;
    moveit_pro::base::RobotStatePtr robot_state_;

    std::string planning_group_name_;
    const moveit_pro::base::JointModelGroup* joint_model_group_;
    std::string tip_link_name_;

    std::unique_ptr<random_numbers::RandomNumberGenerator> generator_;

    bool check_self_collision_{false};
    std::optional<moveit_pro::base::planning_scene::PlanningScene> planning_scene_;

    size_t sample_size_;
    double ik_timeout_;
    size_t ik_iteration_display_step_;
    double success_count_;
    std::vector<int> solve_times_;
    double average_solve_time_;
    double success_rate_;
    bool calculation_done_;

    std::ofstream data_file_;

    void initialize();
    void gather_data();
};
