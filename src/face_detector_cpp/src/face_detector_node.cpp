#include "face_detector_cpp/face_detector.hpp"

/**
 * @brief 主函数入口,创建节点并运行
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto face_detector = std::make_shared<FaceDetectorNode>();
    try
    {
        face_detector->initialize_subscribers();
    }
    catch(const std::exception& e)
    {
        RCLCPP_FATAL(face_detector->get_logger(), "Failed to initialize subscribers: %s", e.what());
        rclcpp::shutdown();
        return -1;
    }
    rclcpp::spin(face_detector);
    rclcpp::shutdown();
    return 0;
}
