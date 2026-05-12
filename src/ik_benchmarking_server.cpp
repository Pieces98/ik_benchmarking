// IK benchmarking action server for the MoveIt Pro environment.
//
// robot_description / robot_description_semantic are read from disk by the
// launch file and pushed onto this node's parameters; on each goal they are
// forwarded as parameter overrides to the IKBenchmarking child node so MoveIt
// Pro's RobotModelLoader builds the model without contacting any external node.

#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "ik_benchmarking/action/ik_benchmark.hpp"
#include "ik_benchmarking/ik_benchmarking.hpp"

using IKBenchmark = ik_benchmarking::action::IKBenchmark;
using GoalHandleIKBenchmark = rclcpp_action::ServerGoalHandle<IKBenchmark>;

class IKBenchmarkingServer : public rclcpp::Node {
   public:
    explicit IKBenchmarkingServer(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("ik_benchmarking_server", options) {
        using namespace std::placeholders;
        action_server_ = rclcpp_action::create_server<IKBenchmark>(
            this, "ik_benchmark",
            std::bind(&IKBenchmarkingServer::handle_goal, this, _1, _2),
            std::bind(&IKBenchmarkingServer::handle_cancel, this, _1),
            std::bind(&IKBenchmarkingServer::handle_accepted, this, _1));
    }

   private:
    rclcpp_action::Server<IKBenchmark>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const IKBenchmark::Goal> goal) {
        RCLCPP_INFO(this->get_logger(),
                    "Received IKBenchmark goal request with solver %s",
                    goal->solver_name.c_str());
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleIKBenchmark> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleIKBenchmark> goal_handle) {
        using namespace std::placeholders;
        bool execute_once{true};
        std::thread{std::bind(&IKBenchmarkingServer::execute, this, _1, _2),
                    goal_handle, execute_once}
            .detach();
    }

    void execute(const std::shared_ptr<GoalHandleIKBenchmark> goal_handle,
                 bool execute_once) {
        RCLCPP_INFO(this->get_logger(), "Executing goal");

        const auto goal = goal_handle->get_goal();

        // robot_description / robot_description_semantic are expected to have been
        // injected by the launch file as parameters on this server.
        if (!this->has_parameter("robot_description") ||
            !this->has_parameter("robot_description_semantic")) {
            RCLCPP_ERROR(this->get_logger(),
                         "robot_description / robot_description_semantic are not set on "
                         "this node. Make sure the launch file injects them.");
            auto failed = std::make_shared<IKBenchmark::Result>();
            failed->calculation_done = false;
            goal_handle->abort(failed);
            if (execute_once) rclcpp::shutdown();
            return;
        }
        const auto rd_param = this->get_parameter("robot_description");
        const auto rds_param = this->get_parameter("robot_description_semantic");
        if (rd_param.get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
            rds_param.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
            RCLCPP_ERROR(this->get_logger(),
                         "robot_description / robot_description_semantic must be string parameters.");
            auto failed = std::make_shared<IKBenchmark::Result>();
            failed->calculation_done = false;
            goal_handle->abort(failed);
            if (execute_once) rclcpp::shutdown();
            return;
        }

        rclcpp::NodeOptions child_options;
        child_options.automatically_declare_parameters_from_overrides(true);
        auto& overrides = child_options.parameter_overrides();

        // Forward only the IKBenchmarking-needed parameters from this server to the
        // child node. Inheriting `this->get_node_options()` would replay launch's
        // `--ros-args --params-file ...` and conflicts with MoveIt Pro's RobotModelLoader
        // (the child ends up sharing the server's executor association).
        const std::vector<std::string> forwarded_keys = {
            "planning_group", "random_seed", "sample_size", "ik_timeout",
            "ik_iteration_display_step", "check_self_collision",
        };
        for (const auto& key : forwarded_keys) {
            if (this->has_parameter(key)) {
                overrides.push_back(this->get_parameter(key));
            }
        }
        overrides.push_back(rd_param);
        overrides.push_back(rds_param);

        IKBenchmarking ik_benchmarker("ik_benchmarker", goal->solver_name.c_str(),
                                      goal->csv_filename.c_str(), child_options);
        ik_benchmarker.run();

        auto result = std::make_shared<IKBenchmark::Result>();
        rclcpp::Rate loop_rate(1);
        while (!ik_benchmarker.calculation_done()) {
            loop_rate.sleep();
        }
        result->calculation_done = true;
        result->success_rate = ik_benchmarker.get_success_rate();
        result->average_solve_time = ik_benchmarker.get_average_solve_time();

        goal_handle->succeed(result);

        if (execute_once) rclcpp::shutdown();
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto action_server = std::make_shared<IKBenchmarkingServer>(node_options);

    RCLCPP_INFO(action_server->get_logger(),
                "IK Benchmarking (MoveIt Pro) action server started.");

    rclcpp::spin(action_server);

    rclcpp::shutdown();
    return 0;
}
