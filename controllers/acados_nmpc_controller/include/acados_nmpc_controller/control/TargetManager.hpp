#ifndef _TARGETMANGER_HPP
#define _TARGETMANGER_HPP
#include "acados_nmpc_controller/control/BsplineTrajectorySource.hpp"
#include "acados_nmpc_controller/control/TargetInfoLoader.hpp"
#include "acados_nmpc_controller/control/TrajectoryGenerator.hpp"
#include "acados_nmpc_controller/utils/Types.hpp"
#include "acados_nmpc_controller/utils/TargetTrajectories.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <memory>
#include <mutex>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <atomic>
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <array>

class TargetManager {
public:
    explicit TargetManager(rclcpp_lifecycle::LifecycleNode::SharedPtr node);
    ~TargetManager() = default;
    void updateTrajectoryBuffer();
    TargetTrajectories getCurrentTrajectorySegment();
    void exit() {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        is_trajectory_updated_ = true;
        is_traj_track_started_ = false;
        current_index_ = 0;
        last_curr_path_publish_time_ns_ = 0;
        publishEmptyPaths();
    }

protected:
    void initializeConfiguredTrajectory(const std::string &trajectory_source);
    bool loadTargetInfo(const std::string &target_file,
                        const std::string &target_name);
    bool setFullTrajectory(const TargetTrajectories &trajectory,
                           const std::string &source_label);
    void targetBsplineCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void publishFullTrajectory() const;
    void publishEmptyPaths() const;
    void initializeTrajectoryWindows();
    std::shared_ptr<TargetTrajectories> makeTrajectoryWindow() const;
    bool shouldPublishCurrentPath(const rclcpp::Time &stamp);
    TrajectoryGenerator makeTrajectoryGenerator() const;

    vector_array_t full_trajectory_{};
    vector_array_t full_inputs_{};
    vector_t full_time_{};
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr currpath_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_bspline_sub_;
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    std::atomic<std::shared_ptr<TargetTrajectories>> trajectory_buffer_; // 线程安全的轨迹缓冲区
    mutable std::mutex trajectory_mutex_;
    long curr_time{0};                                                   // 轨迹开始时间戳，单位ns
    long curr_duration{0};                                               // 从轨迹开始到当前的时间差，单位ns
    int current_index_ = 0;                                              // 当前轨迹的起始索引
    double dt_p{0.025};                                                  // 预测时间步长（25 ms）
    double dt_ = 0.001;                                                  // 控制时间步长（1 ms）
    int k_p = 0;                                                         // 预测控制步长比例
    int horizon_steps_ = 20;                                             // 预测步数（1 秒分为 20 步）
    int path_publish_step_{1};                                           // Full trajectory visualization decimation
    double curr_path_publish_rate_{20.0};                                // Current horizon visualization Hz
    int64_t last_curr_path_publish_time_ns_{0};
    int state_dim_ = 13;
    int input_dim_ = 8;
    int bspline_degree_{3};
    double bspline_duration_{0.0};
    std::string bspline_frame_{"enu"};
    std::string target_file_;
    std::string target_name_;
    std::string path_frame_{"odom"};
    std::array<std::shared_ptr<TargetTrajectories>, 2> trajectory_windows_{};
    size_t active_window_index_{0};
    nav_msgs::msg::Path current_path_msg_;
    TargetInfoLoader target_info_loader_;
    BsplineTrajectorySource bspline_source_;
    bool is_trajectory_updated_{false};                                  // 轨迹是否更新的标志
    bool is_traj_track_started_{false};                                  // 轨迹缓冲区是否更新的标志
};

#endif // _TARGETMANGER_HPP
