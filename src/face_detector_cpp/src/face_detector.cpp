/**
 * 1.接受/camera/camera/color/image_raw话题的图像消息
 * 2.图像浅层处理,
 * 3.调用face_detector进行人脸检测
 * 4.发布检测结果到/face_detector/detections话题
 */

// #include
#include "face_detector_cpp/face_detector.hpp"


#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>


#include <stdexcept>

// 定义构造函数 
FaceDetectorNode::FaceDetectorNode() : Node("face_detector_node")
{   

    using namespace std::chrono_literals;
    // 加载人脸检测分类器
    // std::string default_cascade_path = (std::string)cv::data::haarcascades + "haarcascade_frontalface_default.xml";
    
    // this ->declare_parameter<std::string>("cascade_path", "");
    this ->declare_parameter<std::string>("prototxt_path", "");
    this ->declare_parameter<std::string>("model_path", "");

    this ->declare_parameter<std::string>("input_topic", "/camera/image_raw");
    this ->declare_parameter<std::string>("output_topic", "/face_detector/detections");
    this ->declare_parameter<std::string>("debug_image_topic", "face_debug_image");


    // std::string cascade_file_path = this -> get_parameter("cascade_path").as_string();
    std::string prototxt_path = this -> get_parameter("prototxt_path").as_string();
    std::string model_path = this -> get_parameter("model_path").as_string();


    std::string output_topic = this -> get_parameter("output_topic").as_string();

    debug_image_topic_ = this -> get_parameter("debug_image_topic").as_string();
    input_topic_ = this -> get_parameter("input_topic").as_string();


    net_ = cv::dnn::readNetFromCaffe(prototxt_path, model_path);
    if (net_.empty()) {
        RCLCPP_FATAL(this->get_logger(), "Error loading DNN model files: %s , %s", prototxt_path.c_str(), model_path.c_str());
        throw std::runtime_error("Could not load DNN model files.");
    }
    else {
        RCLCPP_INFO(this->get_logger(), "Successfully loaded DNN model files: %s , %s", prototxt_path.c_str(), model_path.c_str());
    }
    // if (!face_cascade_.load(cascade_file_path)) {
    //     RCLCPP_FATAL(this->get_logger(), "Error loading cascade file at %s", cascade_file_path.c_str());
    //     throw std::runtime_error("Could not load cascade classifier file.");
    // }   
    // else {
    //     RCLCPP_INFO(this->get_logger(), "Successfully loaded cascade file at %s", cascade_file_path.c_str());
    // }

    pub_detections_ = this->create_publisher<vision_msgs::msg::Detection2DArray>(output_topic, 10);
    check_topic_timer_ = this->create_wall_timer(3s, std::bind(&FaceDetectorNode::check_topic_callback, this));
    
};

void FaceDetectorNode::initialize_subscribers()
{
    it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this()); 
    sub_ = it_ -> subscribe(input_topic_, 10, std::bind(&FaceDetectorNode::image_callback, this, std::placeholders::_1));
    pub_debug_image_ = it_ -> advertise(debug_image_topic_, 10);
    RCLCPP_INFO(this->get_logger(), "Ready to subscribed to image topic: %s!", input_topic_.c_str());
}


void FaceDetectorNode::check_topic_callback()
{
    if (!received_image_flag_.load())
    {
        RCLCPP_WARN(this->get_logger(), "No camera data!");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Get subscribed!");
        received_image_flag_ = false;
    }
}

// 定义图像回调函数

void FaceDetectorNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
    received_image_flag_ = true;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge 异常: %s", e.what());
        return;
    }
    cv::Mat& frame = cv_ptr->image;
    cv::Mat debug_image = frame.clone();

    // cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);
    // 直方图 图像中暗的地方变得更暗，亮的地方变得更亮，从而极大地增强了图像的全局对比度。
    // cv::equalizeHist(gray_image, gray_image);

    std::vector<cv::Rect> faces;

    cv::Mat blob = cv::dnn::blobFromImage(frame, 1.0, cv::Size(300, 300), cv::Scalar(104.0, 177.0, 123.0));

    net_.setInput(blob);
    cv::Mat detections = net_.forward();

    

    // face_cascade_.detectMultiScale(gray_image, faces, 1.1, 5, 0, cv::Size(30,30));

    auto detection_array_msg = std::make_unique<vision_msgs::msg::Detection2DArray>();
    detection_array_msg->header = msg->header;


    cv::Mat reshape_detections = detections.reshape(1, detections.size.p[2]);
    // for (const auto& rect : faces)
    // {
    //     vision_msgs::msg::Detection2D detection;
    //     detection.header = msg->header;
    //     detection.bbox.center.position.x = static_cast<double>(rect.x + rect.width / 2);
    //     detection.bbox.center.position.y = static_cast<double>(rect.y + rect.height / 2);
    //     detection.bbox.size_x = static_cast<double>(rect.width);
    //     detection.bbox.size_y = static_cast<double>(rect.height);
    //     vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
    //     hypothesis.hypothesis.class_id = "face";
    //     hypothesis.hypothesis.score = 1.0;
    //     detection.results.push_back(hypothesis);
    //     detection_array_msg->detections.push_back(detection);
    //     cv::rectangle(debug_image, rect, cv::Scalar(0, 255, 0), 2);
    // }
    
    float confidence_threshold = 0.5;

    for (int i = 0; i < reshape_detections.rows; ++i)
    {
        float confidence = reshape_detections.at<float>(i, 2);
        if (confidence > confidence_threshold)
        {
            int x1 = static_cast<int>(reshape_detections.at<float>(i, 3) * frame.cols);   //从第三列获取框体左上角的 x 坐标 * 图像宽度
            int y1 = static_cast<int>(reshape_detections.at<float>(i, 4) * frame.rows);   //从第四列获取框体左上角的 y 坐标 * 图像高度
            int x2 = static_cast<int>(reshape_detections.at<float>(i, 5) * frame.cols);   //从第五列获取框体右下角的 x 坐标 * 图像宽度
            int y2 = static_cast<int>(reshape_detections.at<float>(i, 6) * frame.rows);   //从第六列获取框体右下角的 y 坐标 * 图像高度
            cv::Rect rect(x1, y1, x2 - x1, y2 - y1);  // 创建矩形框 x2 - x1 为宽度, y2 - y1 为高度

            vision_msgs::msg::Detection2D detection;
            detection.header = msg->header;
            detection.bbox.center.position.x = static_cast<double>(rect.x + rect.width / 2);    //中心点 x 坐标
            detection.bbox.center.position.y = static_cast<double>(rect.y + rect.height / 2);   //中心点 y 坐标
            detection.bbox.size_x = static_cast<double>(rect.width);
            detection.bbox.size_y = static_cast<double>(rect.height);
            vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
            hypothesis.hypothesis.class_id = "face";
            hypothesis.hypothesis.score = confidence;
            detection.results.push_back(hypothesis);
            detection_array_msg->detections.push_back(detection);
            cv::rectangle(debug_image, rect, cv::Scalar(0, 255, 0), 2);
            std::string text = "Face:" + std::to_string(confidence*100) + "%";
            cv::putText(debug_image, text, cv::Point(x1, y1 - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);  
        }
    }
    
    if (!detection_array_msg->detections.empty())
    {
        pub_detections_->publish(std::move(detection_array_msg));
    }
        sensor_msgs::msg::Image::SharedPtr debug_msg = cv_bridge::CvImage(msg->header, "bgr8", debug_image).toImageMsg();
        pub_debug_image_.publish(debug_msg);
}