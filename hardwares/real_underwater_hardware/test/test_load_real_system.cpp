#include <gtest/gtest.h>
#include <controller_manager/controller_manager.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>

// 自定义 main，保证 rclcpp::init/shutdown 生命周期正确
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    int ret = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return ret;
}

TEST(ControllerLoadTest, LoadRealSystemController) {
    auto node = rclcpp::Node::make_shared("test_load_real_system_node");

    // 创建controller_manager
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(node);

    // 加载hardware插件
    std::string hardware_plugin = "real_underwater_hardware/RealSystem";
    auto loader = std::make_shared<pluginlib::ClassLoader<hardware_interface::SystemInterface>>(
        "hardware_interface", "hardware_interface::SystemInterface");
    EXPECT_NO_THROW({
        auto instance = loader->createSharedInstance(hardware_plugin);
        EXPECT_NE(instance, nullptr);
    });
}