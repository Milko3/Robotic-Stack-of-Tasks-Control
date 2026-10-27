// sot_controller.cpp - ros2_control controller that runs our two-task Stack of Tasks on the Panda.
//
// Same test as main() in sot.cpp:
//   Task 1 (priority 1): yaw of the hand -> 0           (standard_method::calculateDq1)
//   Task 2 (priority 2): hand to start + goal_offset     (standard_method::calculateDq2)
//
// Every control cycle:
//   read joint positions -> calculateDq1 -> calculateDq2 -> limit speed -> write joint velocities
//
// The SoT code is the repo's sot.cpp, included here with SOT_NO_MAIN so its main() is left out.

#define SOT_NO_MAIN
#include "sot.cpp"

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace sot_ros {

using CallbackReturn = controller_interface::CallbackReturn;

class SotController : public controller_interface::ControllerInterface {
public:
    CallbackReturn on_init() override {
        auto_declare<std::vector<std::string>>("joints", {
            "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
            "panda_joint5", "panda_joint6", "panda_joint7"});
        auto_declare<std::string>("ee_frame", "panda_link7");
        // Goal = hand start position + goal_offset [m] (same as the test in sot.cpp).
        auto_declare<std::vector<double>>("goal_offset", {0.10, 0.0, -0.10});
        // Safety: every joint speed is scaled down to stay below this [rad/s].
        auto_declare<double>("max_joint_speed", 2.0);
        auto_declare<double>("publish_rate", 50.0);   // Hz, for errors + markers
        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration command_interface_configuration() const override {
        controller_interface::InterfaceConfiguration conf;
        conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        for (const auto& joint : jointNames_) {
            conf.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
        }
        return conf;
    }

    controller_interface::InterfaceConfiguration state_interface_configuration() const override {
        controller_interface::InterfaceConfiguration conf;
        conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        for (const auto& joint : jointNames_) {
            conf.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
        }
        return conf;
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
        const auto logger = get_node()->get_logger();
        jointNames_ = get_node()->get_parameter("joints").as_string_array();
        const std::string eeFrameName = get_node()->get_parameter("ee_frame").as_string();
        const auto offset = get_node()->get_parameter("goal_offset").as_double_array();
        maxJointSpeed_ = get_node()->get_parameter("max_joint_speed").as_double();
        const double rate = get_node()->get_parameter("publish_rate").as_double();
        publishPeriod_ = rclcpp::Duration::from_seconds(1.0 / std::max(rate, 1.0));
        if (offset.size() != 3) {
            RCLCPP_ERROR(logger, "goal_offset needs 3 numbers (x, y, z)");
            return CallbackReturn::FAILURE;
        }
        goalOffset_ = Eigen::Vector3d(offset[0], offset[1], offset[2]);

        // Robot model from robot_state_publisher's robot_description.
        std::string urdf;
        if (!getRobotDescription(urdf)) {
            RCLCPP_ERROR(logger, "Could not get robot_description from /robot_state_publisher");
            return CallbackReturn::ERROR;
        }
        try {
            pinocchio::urdf::buildModelFromXML(urdf, model_);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger, "Pinocchio could not parse the URDF: %s", e.what());
            return CallbackReturn::ERROR;
        }
        data_ = pinocchio::Data(model_);
        if (!model_.existFrame(eeFrameName)) {
            RCLCPP_ERROR(logger, "Frame '%s' is not in the URDF", eeFrameName.c_str());
            return CallbackReturn::ERROR;
        }
        eeFrame_ = static_cast<int>(model_.getFrameId(eeFrameName));

        // Where each controlled joint sits in Pinocchio's q and dq vectors.
        qIndex_.clear();
        vIndex_.clear();
        for (const auto& joint : jointNames_) {
            if (!model_.existJointName(joint)) {
                RCLCPP_ERROR(logger, "Joint '%s' is not in the URDF", joint.c_str());
                return CallbackReturn::ERROR;
            }
            const auto id = model_.getJointId(joint);
            qIndex_.push_back(model_.joints[id].idx_q());
            vIndex_.push_back(model_.joints[id].idx_v());
        }

        errorPublisher_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
            "~/task_errors", rclcpp::SystemDefaultsQoS());
        rtErrorPublisher_ = std::make_unique<
            realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(errorPublisher_);
        markerPublisher_ = get_node()->create_publisher<visualization_msgs::msg::MarkerArray>(
            "~/markers", rclcpp::SystemDefaultsQoS());
        rtMarkerPublisher_ = std::make_unique<
            realtime_tools::RealtimePublisher<visualization_msgs::msg::MarkerArray>>(markerPublisher_);

        RCLCPP_INFO(logger, "SoT controller configured: %zu joints, goal offset (%.2f, %.2f, %.2f) m",
                    jointNames_.size(), goalOffset_.x(), goalOffset_.y(), goalOffset_.z());
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State&) override {
        q_ = pinocchio::neutral(model_);
        if (!readPositions()) {
            RCLCPP_ERROR(get_node()->get_logger(), "Joint positions are not available");
            return CallbackReturn::ERROR;
        }
        // Goal = where the hand is now + offset (same as the test in sot.cpp).
        pinocchio::computeJointJacobians(model_, data_, q_);
        pinocchio::updateFramePlacements(model_, data_);
        goalXYZ_ = data_.oMf[eeFrame_].translation() + goalOffset_;
        started_ = false;
        RCLCPP_INFO(get_node()->get_logger(), "SoT controller active, goal (%.3f, %.3f, %.3f) m",
                    goalXYZ_.x(), goalXYZ_.y(), goalXYZ_.z());
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override {
        for (auto& command : command_interfaces_) command.set_value(0.0);
        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration&) override {
        if (!started_) {
            startTime_ = time;
            lastPublish_ = time;
            started_ = true;
        }
        if (!readPositions()) return controller_interface::return_type::ERROR;

        // Our two tasks, exactly as in sot.cpp's test loop.
        Eigen::VectorXd dq1 = sot_.calculateDq1(model_, data_, q_, eeFrame_);
        Eigen::VectorXd dq2 = sot_.calculateDq2(model_, data_, q_, eeFrame_, goalXYZ_, dq1);
        if (!dq2.allFinite()) dq2.setZero();

        // Safety: scale all joint speeds down together if one is too fast.
        const double fastest = dq2.cwiseAbs().maxCoeff();
        if (fastest > maxJointSpeed_) dq2 *= maxJointSpeed_ / fastest;

        for (size_t i = 0; i < command_interfaces_.size() && i < vIndex_.size(); ++i) {
            command_interfaces_[i].set_value(dq2[vIndex_[i]]);
        }

        if ((time - lastPublish_) >= publishPeriod_) {
            lastPublish_ = time;
            publish((time - startTime_).seconds());
        }
        return controller_interface::return_type::OK;
    }

private:
    bool readPositions() {
        for (size_t i = 0; i < state_interfaces_.size() && i < qIndex_.size(); ++i) {
            const double value = state_interfaces_[i].get_value();
            if (!std::isfinite(value)) return false;
            q_[qIndex_[i]] = value;
        }
        return state_interfaces_.size() == qIndex_.size();
    }

    bool getRobotDescription(std::string& urdf) {
        // A separate helper node: the controller's own node already spins in the
        // controller manager's executor and cannot be spun by this client.
        auto helper = std::make_shared<rclcpp::Node>(
            "sot_controller_description_client",
            rclcpp::NodeOptions().start_parameter_services(false).start_parameter_event_publisher(false));
        auto client = std::make_shared<rclcpp::SyncParametersClient>(helper, "/robot_state_publisher");
        for (int attempt = 0; attempt < 30 && !client->wait_for_service(1s); ++attempt) {
            if (!rclcpp::ok()) return false;
            RCLCPP_INFO(get_node()->get_logger(), "Waiting for /robot_state_publisher ...");
        }
        if (!client->service_is_ready()) return false;
        for (const auto& parameter : client->get_parameters({"robot_description"})) {
            if (parameter.get_name() == "robot_description") urdf = parameter.as_string();
        }
        return !urdf.empty();
    }

    // Errors as [t, yaw error (rad), position error (m)] and RViz markers.
    void publish(double t) {
        const double yawError = sot_.get_dTheta(data_, eeFrame_);
        const Eigen::Vector3d hand = data_.oMf[eeFrame_].translation();
        const double posError = (goalXYZ_ - hand).norm();

        if (rtErrorPublisher_ && rtErrorPublisher_->trylock()) {
            rtErrorPublisher_->msg_.data = {t, yawError, posError};
            rtErrorPublisher_->unlockAndPublish();
        }
        if (rtMarkerPublisher_ && rtMarkerPublisher_->trylock()) {
            auto& markers = rtMarkerPublisher_->msg_.markers;
            markers.resize(2);
            const auto stamp = get_node()->now();

            // 0: goal point (orange sphere)
            auto& goal = markers[0];
            goal.header.frame_id = "world";
            goal.header.stamp = stamp;
            goal.ns = "sot";
            goal.id = 0;
            goal.type = visualization_msgs::msg::Marker::SPHERE;
            goal.action = visualization_msgs::msg::Marker::ADD;
            goal.pose.position.x = goalXYZ_.x();
            goal.pose.position.y = goalXYZ_.y();
            goal.pose.position.z = goalXYZ_.z();
            goal.pose.orientation.w = 1.0;
            goal.scale.x = goal.scale.y = goal.scale.z = 0.03;
            goal.color.r = 0.85; goal.color.g = 0.37; goal.color.b = 0.01; goal.color.a = 1.0;

            // 1: line from hand to goal (shows the remaining position error)
            auto& line = markers[1];
            line.header = goal.header;
            line.ns = "sot";
            line.id = 1;
            line.type = visualization_msgs::msg::Marker::LINE_LIST;
            line.action = visualization_msgs::msg::Marker::ADD;
            line.pose.orientation.w = 1.0;
            line.scale.x = 0.004;
            line.color.r = 0.2; line.color.g = 0.2; line.color.b = 0.2; line.color.a = 1.0;
            line.points.resize(2);
            line.points[0].x = hand.x(); line.points[0].y = hand.y(); line.points[0].z = hand.z();
            line.points[1].x = goalXYZ_.x(); line.points[1].y = goalXYZ_.y(); line.points[1].z = goalXYZ_.z();

            rtMarkerPublisher_->unlockAndPublish();
        }
    }

    std::vector<std::string> jointNames_;
    std::vector<int> qIndex_, vIndex_;
    pinocchio::Model model_;
    pinocchio::Data data_;
    int eeFrame_ = 0;
    Eigen::VectorXd q_;
    Eigen::Vector3d goalOffset_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d goalXYZ_ = Eigen::Vector3d::Zero();
    double maxJointSpeed_ = 2.0;
    standard_method sot_;

    bool started_ = false;
    rclcpp::Time startTime_, lastPublish_;
    rclcpp::Duration publishPeriod_{0, 20000000};

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr errorPublisher_;
    std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> rtErrorPublisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markerPublisher_;
    std::unique_ptr<realtime_tools::RealtimePublisher<visualization_msgs::msg::MarkerArray>> rtMarkerPublisher_;
};

}  // namespace sot_ros

PLUGINLIB_EXPORT_CLASS(sot_ros::SotController, controller_interface::ControllerInterface)
