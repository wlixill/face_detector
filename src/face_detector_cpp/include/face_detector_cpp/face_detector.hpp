#ifndef FACE_DETECTOR_HPP
#define FACE_DETECTOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <opencv2/opencv.hpp>

#include <opencv2/dnn/dnn.hpp>


#include <chrono>
#include <atomic>

/**
 * @class FaceDetectorNode
 * @brief 人脸检测节点类，继承自rclcpp::Node
 */
class FaceDetectorNode : public rclcpp::Node
{
public:
/**
 * @brief 构造函数
 */
    FaceDetectorNode();

    void initialize_subscribers();
private:
/**
 * @brief 图像回调函数
 * 
 * @param msg 传入ROS图像
 */
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
/**
 * @brief 检查输入话题是否有订阅者
 * 
 */
    void check_topic_callback();

    // ROS2成员
    std::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Subscriber sub_;  // 输入图像订阅者
    image_transport::Publisher pub_debug_image_;   // 输出图像发布者
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_detections_; // 人脸检测结果发布者

    // cv::CascadeClassifier face_cascade_; // OpenCV人脸检测分类器
    cv::dnn::Net net_; // DNN网络模型

    rclcpp::TimerBase::SharedPtr check_topic_timer_;
    std::atomic<bool> received_image_flag_{false};  //标志位默认为false
    
    std::string input_topic_;
    std::string debug_image_topic_;
};




#endif // FACE_DETECTOR_HPP