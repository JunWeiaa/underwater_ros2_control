// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <unistd.h>

#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gz/sim/components/ChildLinkName.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/Model.hh>
#include <gz/plugin/Register.hh>

#include <gz/math/Vector3.hh>

#include <Eigen/Eigen>
#include <controller_manager/controller_manager.hpp>

#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/component_parser.hpp>

#include <pluginlib/class_loader.hpp>

#include <rclcpp/rclcpp.hpp>

#include "gz_underwater_hardware/gz_underwater_plugin.hpp"
#include "gz_underwater_hardware/gz_system.hpp"

namespace gz_underwater_hardware {
class GZResourceManager : public hardware_interface::ResourceManager {
public:
    GZResourceManager(
        rclcpp::Node::SharedPtr &node,
        sim::EntityComponentManager &ecm,
        std::map<std::string, sim::Entity> enabledJoints,
        const Eigen::VectorXd &propellerDiameter,
        const Eigen::VectorXd &thrustCoefficient) :
        ResourceManager(
            node->get_node_clock_interface(),
            node->get_node_logging_interface()),
        gz_system_loader_("gz_underwater_hardware", "gz_underwater_hardware::GazeboSimSystemInterface"),
        logger_(node->get_logger().get_child("GZResourceManager")) {
        node_ = node;
        ecm_ = &ecm;
        enabledJoints_ = enabledJoints;
        propellerDiameter_ = propellerDiameter;
        thrustCoefficient_ = thrustCoefficient;
    }

    GZResourceManager(const GZResourceManager &) = delete;

    // Called from Controller Manager when robot description is initialized from callback
    bool load_and_initialize_components(
        const std::string &urdf,
        unsigned int update_rate) override {
        components_are_loaded_and_initialized_ = true;

        const auto hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf);

        for (const auto &individual_hardware_info : hardware_info) {
            std::string robot_hw_sim_type_str_ = individual_hardware_info.hardware_plugin_name;
            RCLCPP_DEBUG(logger_, "Load hardware interface %s ...", robot_hw_sim_type_str_.c_str());

            // Load hardware
            std::unique_ptr<GazeboSimSystemInterface> gzSimSystem;
            std::scoped_lock guard(resource_interfaces_lock_, claimed_command_interfaces_lock_);
            try {
                gzSimSystem = std::unique_ptr<GazeboSimSystemInterface>(
                    gz_system_loader_.createUnmanagedInstance(robot_hw_sim_type_str_));
            } catch (pluginlib::PluginlibException &ex) {
                RCLCPP_ERROR(
                    logger_,
                    "The plugin failed to load for some reason. Error: %s\n",
                    ex.what());
                continue;
            }

            // initialize simulation requirements
            if (!gzSimSystem->initSim(
                    node_,
                    enabledJoints_,
                    thrustCoefficient_,
                    propellerDiameter_,
                    individual_hardware_info,
                    *ecm_,
                    update_rate)) {
                RCLCPP_FATAL(
                    logger_, "Could not initialize robot simulation interface");
                components_are_loaded_and_initialized_ = false;
                break;
            }
            RCLCPP_DEBUG(
                logger_, "Initialized robot simulation interface %s!", robot_hw_sim_type_str_.c_str());

            // initialize hardware
            import_component(std::move(gzSimSystem), individual_hardware_info);
        }

        return components_are_loaded_and_initialized_;
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    sim::EntityComponentManager *ecm_;
    std::map<std::string, sim::Entity> enabledJoints_;
    Eigen::VectorXd propellerDiameter_;
    Eigen::VectorXd thrustCoefficient_;

    /// \brief Interface loader
    pluginlib::ClassLoader<GazeboSimSystemInterface> gz_system_loader_;

    rclcpp::Logger logger_;
};

//////////////////////////////////////////////////
class GazeboSimUnderwaterPluginPrivate {
public:
    /// \brief Get a list of enabled, unique, 1-axis joints of the model. If no
    /// joint names are specified in the plugin configuration, all valid 1-axis
    /// joints are returned
    /// \param[in] _entity Entity of the model that the plugin is being
    /// configured for
    /// \param[in] _ecm Gazebo Entity Component Manager
    /// \return List of entities containing all enabled joints
    std::map<std::string, sim::Entity> GetEnabledJoints(
        const sim::Entity &_entity,
        sim::EntityComponentManager &_ecm) const;

    /// \brief Convert angular velocity to thrust
    double AngularVelToThrust(double _angVel);

    /// \brief Convert thrust to angular velocity
    double ThrustToAngularVec(double _thrust);

    /// \brief Entity ID for sensor within Gazebo.
    sim::Entity entity_;

    /// \brief Entity ID for link within Gazebo.
    sim::Entity linkEntity;

    /// \brief Node Handles
    std::shared_ptr<rclcpp::Node> node_{nullptr};

    /// \brief Thread where the executor will spin
    std::thread thread_executor_spin_;

    /// \brief Executor to spin the controller
    rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;

    /// \brief Timing
    rclcpp::Duration control_period_ = rclcpp::Duration(1, 0);

    /// \brief Controller manager
    std::shared_ptr<controller_manager::ControllerManager>
        controller_manager_{nullptr};

    /// \brief Last time the update method was called
    rclcpp::Time last_update_sim_time_ros_ =
        rclcpp::Time(static_cast<int64_t>(0), RCL_ROS_TIME);

    /// \brief ECM pointer
    sim::EntityComponentManager *ecm{nullptr};

    /// \brief Flag to indicate if the plugin is being destroyed
    std::atomic<bool> is_destroying_{false};

    /// \brief controller update rate
    int update_rate;

    /// \brief Density of fluid in kgm^-3, default: 1000kgm^-3
    double fluidDensity = 1000;

    /// \brief Number of thrusters, default: 8
    int Thruster_num = 8;

    /// \brief Diameter of propeller in m, default: 0.02
    Eigen::VectorXd propellerDiameter;

    /// \brief Thrust coefficient, default: 1
    Eigen::VectorXd thrustCoefficient;
};

//////////////////////////////////////////////////
std::map<std::string, sim::Entity>
GazeboSimUnderwaterPluginPrivate::GetEnabledJoints(
    const sim::Entity &_entity,
    sim::EntityComponentManager &_ecm) const {
    std::map<std::string, sim::Entity> output;

    std::vector<std::string> enabledJoints;
    const auto model = sim::Model(_entity);
    // Get all available joints
    auto jointEntities = _ecm.ChildrenByComponents(_entity, sim::components::Joint());

    // Iterate over all joints and verify whether they can be enabled or not
    for (const auto &jointEntity : jointEntities) {
        const auto jointName = _ecm.Component<sim::components::Name>(
                                       jointEntity)
                                   ->Data();

        // this->dataPtr->linkEntity = model.LinkByName(_ecm, childLink->Data());

        // Make sure the joint type is supported, i.e. it has exactly one
        // actuated axis
        const auto *jointType = _ecm.Component<sim::components::JointType>(jointEntity);
        switch (jointType->Data()) {
        case sdf::JointType::PRISMATIC:
        case sdf::JointType::REVOLUTE:
        case sdf::JointType::CONTINUOUS:
        case sdf::JointType::GEARBOX: {
            // Supported joint type
            break;
        }
        case sdf::JointType::FIXED: {
            RCLCPP_INFO(
                node_->get_logger(),
                "[gz_underwater_hardware] Fixed joint [%s] (Entity=%lu)] is skipped",
                jointName.c_str(),
                jointEntity);
            continue;
        }
        case sdf::JointType::REVOLUTE2:
        case sdf::JointType::SCREW:
        case sdf::JointType::BALL:
        case sdf::JointType::UNIVERSAL: {
            RCLCPP_WARN(
                node_->get_logger(),
                "[gz_underwater_hardware] Joint [%s] (Entity=%lu)] is of unsupported type."
                " Only joints with a single axis are supported.",
                jointName.c_str(),
                jointEntity);
            continue;
        }
        default: {
            RCLCPP_WARN(
                node_->get_logger(),
                "[gz_underwater_hardware] Joint [%s] (Entity=%lu)] is of unknown type",
                jointName.c_str(),
                jointEntity);
            continue;
        }
        }
        output[jointName] = jointEntity;
    }

    return output;
}

GazeboSimUnderwaterPlugin::GazeboSimUnderwaterPlugin() :
    dataPtr(std::make_unique<GazeboSimUnderwaterPluginPrivate>()) {
}

//////////////////////////////////////////////////
GazeboSimUnderwaterPlugin::~GazeboSimUnderwaterPlugin() {
    // Set destroying flag first to prevent further updates
    this->dataPtr->is_destroying_.store(true);

    // Stop controller manager thread
    if (!this->dataPtr->controller_manager_) {
        return;
    }

    // Remove node from executor before canceling
    this->dataPtr->executor_->remove_node(this->dataPtr->controller_manager_);

    // Cancel executor to stop spinning
    this->dataPtr->executor_->cancel();

    // Wait for thread to finish
    if (this->dataPtr->thread_executor_spin_.joinable()) {
        this->dataPtr->thread_executor_spin_.join();
    }

    // Reset shared pointers to ensure proper cleanup
    this->dataPtr->controller_manager_.reset();
    this->dataPtr->executor_.reset();
    this->dataPtr->node_.reset();

    // Log cleanup completion
    RCLCPP_INFO(
        rclcpp::get_logger("GazeboSimUnderwaterPlugin"),
        "GazeboSimUnderwaterPlugin destroyed and cleaned up successfully");
}

//////////////////////////////////////////////////
void GazeboSimUnderwaterPlugin::Configure(
    const sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    sim::EntityComponentManager &_ecm,
    sim::EventManager &) {
    rclcpp::Logger logger = rclcpp::get_logger("GazeboSimROS2ControlPlugin");
    // Make sure the controller is attached to a valid model
    const auto model = sim::Model(_entity);

    if (!model.Valid(_ecm)) {
        RCLCPP_ERROR(
            logger,
            "[Gazebo ROS 2 Control] Failed to initialize because [%s] (Entity=%lu)] is not a model."
            "Please make sure that Gazebo ROS 2 Control is attached to a valid model.",
            model.Name(_ecm).c_str(),
            _entity);
        return;
    }

    // Get params from SDF
    auto param_file_name = _sdf->Get<std::string>("parameters");

    if (param_file_name.empty()) {
        RCLCPP_ERROR(
            logger,
            "Gazebo Underwater ros2 control found an empty parameters file. Failed to initialize.");
        return;
    }

    // Get params from SDF
    std::vector<std::string> arguments = {"--ros-args"};

    auto sdfPtr = const_cast<sdf::Element *>(_sdf.get());

    sdf::ElementPtr argument_sdf = sdfPtr->GetElement("parameters");
    while (argument_sdf) {
        std::string argument = argument_sdf->Get<std::string>();
        arguments.push_back(RCL_PARAM_FILE_FLAG);
        arguments.push_back(argument);
        argument_sdf = argument_sdf->GetNextElement("parameters");
    }

    // Get controller manager node name
    std::string controllerManagerNodeName{"controller_manager"};

    if (sdfPtr->HasElement("controller_manager_name")) {
        controllerManagerNodeName = sdfPtr->GetElement("controller_manager_name")->Get<std::string>();
    }

    std::string ns = "/";

    if (sdfPtr->HasElement("ros")) {
        sdf::ElementPtr sdfRos = sdfPtr->GetElement("ros");

        // Set namespace if tag is present
        if (sdfRos->HasElement("namespace")) {
            ns = sdfRos->GetElement("namespace")->Get<std::string>();
            // prevent exception: namespace must be absolute, it must lead with a '/'
            if (ns.empty() || ns[0] != '/') {
                ns = '/' + ns;
            }
        }

        // Get list of remapping rules from SDF
        if (sdfRos->HasElement("remapping")) {
            sdf::ElementPtr argument_sdf = sdfRos->GetElement("remapping");

            arguments.push_back(RCL_ROS_ARGS_FLAG);
            while (argument_sdf) {
                auto argument = argument_sdf->Get<std::string>();
                arguments.push_back(RCL_REMAP_FLAG);
                arguments.push_back(argument);
                argument_sdf = argument_sdf->GetNextElement("remapping");
            }
        }
    }
    if (!_sdf->HasElement("thruster_num")) {
        gzerr << "Missing <thruster_num>. Plugin won't be initialized."
              << std::endl;
        return;
    }
    this->dataPtr->Thruster_num = sdfPtr->GetElement("thruster_num")->Get<int>();

    if (!_sdf->HasElement("thruster_coefficient")) {
        gzerr << "Missing <thruster_coefficient>. Plugin won't be initialized."
              << std::endl;
        return;
    }
    auto coeffStr = sdfPtr->GetElement("thruster_coefficient")->Get<std::string>();
    std::istringstream coeffStream(coeffStr);
    std::vector<double> coeffValues;
    double value;
    // 将字符串解析为 double 值
    while (coeffStream >> value) {
        coeffValues.push_back(value);
    }
    if (static_cast<int>(coeffValues.size()) != this->dataPtr->Thruster_num) {
        gzerr << "<thruster_coefficient> has " << coeffValues.size()
              << " values but <thruster_num> is " << this->dataPtr->Thruster_num
              << ". Plugin won't be initialized." << std::endl;
        return;
    }
    this->dataPtr->thrustCoefficient = Eigen::VectorXd::Map(coeffValues.data(), coeffValues.size());

    if (!_sdf->HasElement("propeller_diameter")) {
        gzerr << "Missing <propeller_diameter>. Plugin won't be initialized."
              << std::endl;
        return;
    }
    auto propellerDiameterStr = sdfPtr->GetElement("propeller_diameter")->Get<std::string>();
    std::istringstream propStream(propellerDiameterStr);
    std::vector<double> propValues;
    double value2;
    // 将字符串解析为 double 值
    while (propStream >> value2) {
        propValues.push_back(value2);
    }
    if (static_cast<int>(propValues.size()) != this->dataPtr->Thruster_num) {
        gzerr << "<propeller_diameter> has " << propValues.size()
              << " values but <thruster_num> is " << this->dataPtr->Thruster_num
              << ". Plugin won't be initialized." << std::endl;
        return;
    }
    this->dataPtr->propellerDiameter = Eigen::VectorXd::Map(propValues.data(), propValues.size());

    std::vector<const char *> argv;
    for (const auto &arg : arguments) {
        argv.push_back(arg.data());
    }
    // Create a default context, if not already
    if (!rclcpp::ok()) {
        init(
            static_cast<int>(argv.size()), argv.data(), rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    }
    // Get link entity
    // auto childLink =
    //     _ecm.Component<gz::sim::components::ChildLinkName>(
    //         model.Entity());
    // this->dataPtr->linkEntity = model.LinkByName(_ecm, childLink->Data());

    std::string node_name = "gz_underwater_control";
    rclcpp::NodeOptions node_options;
    node_options.arguments(arguments);
    node_options.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
    this->dataPtr->node_ = rclcpp::Node::make_shared(node_name, ns, node_options);
    this->dataPtr->executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    this->dataPtr->executor_->add_node(this->dataPtr->node_);
    auto spin = [this] {
        this->dataPtr->executor_->spin();
    };
    this->dataPtr->thread_executor_spin_ = std::thread(spin);

    RCLCPP_DEBUG_STREAM(
        this->dataPtr->node_->get_logger(), "[Gazebo Underwater ROS2 Control] Setting up controller for [" << model.Name(_ecm) << "] (Entity=" << _entity << ")].");

    // Get list of enabled joints
    auto enabledJoints = this->dataPtr->GetEnabledJoints(
        _entity,
        _ecm);

    if (enabledJoints.size() == 0) {
        RCLCPP_DEBUG_STREAM(
            this->dataPtr->node_->get_logger(),
            "[Gazebo Underwater ROS2 Control] There are no available Joints.");
        return;
    }

    std::unique_ptr<hardware_interface::ResourceManager> resource_manager_ =
        std::make_unique<GZResourceManager>(this->dataPtr->node_, _ecm, enabledJoints, this->dataPtr->propellerDiameter, this->dataPtr->thrustCoefficient);

    // Create the controller manager
    RCLCPP_INFO(this->dataPtr->node_->get_logger(), "Loading controller_manager");
    rclcpp::NodeOptions options = controller_manager::get_cm_node_options();
    options.arguments(arguments);
    options.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
    arguments.push_back("-r");
    arguments.push_back("__node:=" + controllerManagerNodeName);
    arguments.push_back("-r");
    arguments.push_back("__ns:=" + ns);
    options.arguments(arguments);

    this->dataPtr->controller_manager_.reset(
        new controller_manager::ControllerManager(
            std::move(resource_manager_),
            this->dataPtr->executor_,
            controllerManagerNodeName,
            this->dataPtr->node_->get_namespace(),
            options));
    this->dataPtr->executor_->add_node(this->dataPtr->controller_manager_);

    this->dataPtr->update_rate = this->dataPtr->controller_manager_->get_update_rate();
    this->dataPtr->control_period_ =
        rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / static_cast<double>(this->dataPtr->update_rate))));

    // // Force setting of use_sim_time parameter
    // this->dataPtr->controller_manager_->set_parameter(
    //     rclcpp::Parameter("use_sim_time", rclcpp::ParameterValue(true)));

    // Wait for CM to receive robot description from the topic and then initialize Resource Manager
    while (!this->dataPtr->controller_manager_->is_resource_manager_initialized()) {
        RCLCPP_WARN(
            this->dataPtr->node_->get_logger(),
            "Waiting RM to load and initialize hardware...");
        std::this_thread::sleep_for(std::chrono::microseconds(2000000));
    }

    this->dataPtr->entity_ = _entity;
}

//////////////////////////////////////////////////
void GazeboSimUnderwaterPlugin::PreUpdate(
    const sim::UpdateInfo &_info,
    sim::EntityComponentManager & /*_ecm*/) {
    if (!this->dataPtr->controller_manager_) {
        return;
    }
    static bool warned{false};
    if (!warned) {
        rclcpp::Duration gazebo_period(_info.dt);

        if (this->dataPtr->control_period_ < _info.dt) {
            RCLCPP_ERROR_STREAM(
                this->dataPtr->node_->get_logger(),
                "Desired controller update period (" << this->dataPtr->control_period_.seconds() << " s) is faster than the gazebo simulation period (" << gazebo_period.seconds() << " s).");
        } else if (this->dataPtr->control_period_ > gazebo_period) {
            RCLCPP_WARN_STREAM(
                this->dataPtr->node_->get_logger(),
                " Desired controller update period (" << this->dataPtr->control_period_.seconds() << " s) is slower than the gazebo simulation period (" << gazebo_period.seconds() << " s).");
        }
        warned = true;
    }

    rclcpp::Time sim_time_ros = this->dataPtr->node_->get_clock()->now();
    const rclcpp::Duration sim_period = sim_time_ros - this->dataPtr->last_update_sim_time_ros_;
    this->dataPtr->controller_manager_->write(sim_time_ros, sim_period);
}

//////////////////////////////////////////////////
void GazeboSimUnderwaterPlugin::PostUpdate(
    const sim::UpdateInfo & /*_info*/,
    const sim::EntityComponentManager & /*_ecm*/) {
    // Early return if plugin is being destroyed
    if (this->dataPtr->is_destroying_.load()) {
        return;
    }

    if (!this->dataPtr->controller_manager_) {
        return;
    }

    // 用 node 的 clock 获取时间，确保 clock_type 一致
    rclcpp::Time sim_time_ros = this->dataPtr->node_->get_clock()->now();

    // 首次调用时初始化时间
    if (this->dataPtr->last_update_sim_time_ros_.nanoseconds() <= 0) {
        this->dataPtr->last_update_sim_time_ros_ = sim_time_ros;
        return;
    }

    // 安全计算时间差（避免回退时间）
    const int64_t period_ns = sim_time_ros.nanoseconds() - this->dataPtr->last_update_sim_time_ros_.nanoseconds();
    if (period_ns < 0) {
        gzwarn << "Detected time jump backward. Resetting last update time.\n";
        this->dataPtr->last_update_sim_time_ros_ = sim_time_ros;
        return;
    }

    const rclcpp::Duration sim_period(period_ns, sim_time_ros.get_clock_type());

    // 检查控制周期
    if (sim_period >= this->dataPtr->control_period_) {
        try {
            this->dataPtr->last_update_sim_time_ros_ = sim_time_ros;
            this->dataPtr->controller_manager_->read(sim_time_ros, sim_period);
            this->dataPtr->controller_manager_->update(sim_time_ros, sim_period);
        } catch (const std::exception &e) {
            gzerr << "Controller update failed: " << e.what() << "\n";
        }
    }
}
} // namespace gz_underwater_hardware

GZ_ADD_PLUGIN(
    gz_underwater_hardware::GazeboSimUnderwaterPlugin,
    gz::sim::System,
    gz_underwater_hardware::GazeboSimUnderwaterPlugin::ISystemConfigure,
    gz_underwater_hardware::GazeboSimUnderwaterPlugin::ISystemPreUpdate,
    gz_underwater_hardware::GazeboSimUnderwaterPlugin::ISystemPostUpdate)
