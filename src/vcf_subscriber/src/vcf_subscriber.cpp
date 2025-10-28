// #include <memory>
// #include <functional>
// #include <string>
// #include <iostream>
// #include <vector>
// #include <opencv2/opencv.hpp>
// #include <fcntl.h>    // O_RDONLY
// #include <sys/mman.h> // mmap
// #include <unistd.h>   // ftruncate, close
// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/image.hpp"
// #include "cv_bridge/cv_bridge.h"
// #include <sensor_msgs/msg/compressed_image.hpp>
// #include "image_transport/image_transport.hpp"
// #include "rclcpp/qos.hpp"

// using std::placeholders::_1;

// #define WIDTH 1920
// #define HEIGHT 1080
// class VCFSubscriber : public rclcpp::Node
// {
// public:
//   VCFSubscriber()
//       : Node("VCF_subscriber")
//   {
//     rclcpp::QoS qos_settings(1); // 参数1表示队列大小
//     qos_settings.reliable()      // 设置可靠性
//         .durability_volatile()   // 设置持久性：非持久性
//         .best_effort();
//     // 创建图像订阅者，订阅名为"vcf_image"的主题
//     subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
//         "/image_raw/compressed", qos_settings, std::bind(&VCFSubscriber::image_callback, this, _1));
//     subscription_depth = this->create_subscription<sensor_msgs::msg::Image>(
//         "/image_raw/compressed/depth", qos_settings, std::bind(&VCFSubscriber::image_callback2, this, _1));
//   }
//   ~VCFSubscriber()
//   {
//     // 析构函数中不需要额外清理，因为订阅者会自动释放资源
//   }

// private:
//   void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
//   {

//     // 使用cv_bridge将ROS图像消息转换为OpenCV图像
//     cv_bridge::CvImagePtr cv_ptr;
//     try
//     {
//       cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8); // 假设图像编码为RGB8
//     }
//     catch (cv_bridge::Exception &e)
//     {
//       RCLCPP_ERROR(this->get_logger(), "图像转换失败: %s", e.what());
//       return;
//     }
//     cv::Mat resized_image;
//     cv::resize(cv_ptr->image, resized_image, cv::Size(1280, 720));
//     cv::cvtColor(resized_image, resized_image, cv::COLOR_BGR2RGB);
//     cv::imshow("VCF Image", resized_image);
//     cv::waitKey(1); // 等待1毫秒以更新窗口
//   }

//   void image_callback2(const sensor_msgs::msg::Image::SharedPtr msg)
//   {
//     cv::Mat depth_image;
//     depth_image = cv::Mat(480, 640, CV_32FC1, const_cast<uchar *>(msg->data.data()));

//     cv::Mat resized_image;
//     cv::resize(depth_image, resized_image, cv::Size(1280, 720));
//     // 将深度图像转换为伪彩色图像
//     double minVal, maxVal;
//     cv::minMaxLoc(resized_image, &minVal, &maxVal);
//     cv::Mat depth_colored;
//     resized_image.convertTo(depth_colored, CV_8UC1, 255 / (maxVal - minVal), -minVal * 255 / (maxVal - minVal));
//     cv::applyColorMap(depth_colored, depth_colored, cv::COLORMAP_JET);
//     cv::imshow("Depth Image", depth_colored);
//     cv::waitKey(1); // 等待1毫秒以更新窗口
//   }

//   rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
//   rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_depth;
// };

// int main(int argc, char *argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<VCFSubscriber>());
//   rclcpp::shutdown();
//   return 0;
// }

#include <iostream>
#include <memory>
#include <chrono>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <sensor_msgs/msg/compressed_image.hpp>
#include "image_transport/image_transport.hpp"
#include "rclcpp/qos.hpp"

using std::placeholders::_1;

#define WIDTH 1920
#define HEIGHT 1080

class VCFSubscriber : public rclcpp::Node
{
public:
    VCFSubscriber()
    : Node("VCF_subscriber")
    {
        RCLCPP_INFO(this->get_logger(), "开始初始化VCF订阅者节点...");
        
        rclcpp::QoS qos_settings(1); // 参数1表示队列大小
        qos_settings.reliable() // 设置可靠性
                   .durability_volatile() // 设置持久性：非持久性
                   .best_effort();
        
        // 创建图像订阅者，订阅名为"vcf_image"的主题
        subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/image_raw/compressed", qos_settings, 
            std::bind(&VCFSubscriber::image_callback, this, _1));
        
        RCLCPP_INFO(this->get_logger(), "已创建彩色图像订阅者，主题: /image_raw/compressed");
        
        subscription_depth = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw/compressed/depth", qos_settings, 
            std::bind(&VCFSubscriber::image_callback2, this, _1));
        
        RCLCPP_INFO(this->get_logger(), "已创建深度图像订阅者，主题: /image_raw/compressed/depth");
        RCLCPP_INFO(this->get_logger(), "节点初始化完成，等待图像数据...");
        
        // 添加定时器，定期输出节点状态
        timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&VCFSubscriber::timer_callback, this));
            
        color_count_ = 0;
        depth_count_ = 0;
    }
    
    ~VCFSubscriber()
    {
        RCLCPP_INFO(this->get_logger(), "VCF订阅者节点关闭");
    }

private:
    void timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "节点运行状态 - 彩色图像接收: %d 帧, 深度图像接收: %d 帧", 
                   color_count_, depth_count_);
    }

    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        color_count_++;
        if (color_count_ % 30 == 0) { // 每30帧输出一次日志，避免太频繁
            RCLCPP_INFO(this->get_logger(), "收到彩色图像 #%d, 时间戳: %ld.%09ld", 
                       color_count_, msg->header.stamp.sec, msg->header.stamp.nanosec);
        } else {
            RCLCPP_DEBUG(this->get_logger(), "收到彩色图像 #%d", color_count_);
        }

        // 使用cv_bridge将ROS图像消息转换为OpenCV图像
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
            RCLCPP_DEBUG(this->get_logger(), "彩色图像转换成功，尺寸: %dx%d", 
                        cv_ptr->image.cols, cv_ptr->image.rows);
        } catch (cv_bridge::Exception & e) {
            RCLCPP_ERROR(this->get_logger(), "彩色图像转换失败: %s", e.what());
            return;
        }
        
        cv::Mat resized_image;
        cv::resize(cv_ptr->image, resized_image, cv::Size(1280, 720));
        cv::cvtColor(resized_image, resized_image, cv::COLOR_BGR2RGB);
        
        RCLCPP_DEBUG(this->get_logger(), "彩色图像已调整尺寸: 1280x720");
        
        cv::imshow("VCF Image", resized_image);
        int key = cv::waitKey(1);
        if (key == 27) { // ESC键
            RCLCPP_INFO(this->get_logger(), "用户按下ESC键，准备退出...");
            rclcpp::shutdown();
        }
    }

    void image_callback2(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        depth_count_++;
        if (depth_count_ % 30 == 0) { // 每30帧输出一次日志
            RCLCPP_INFO(this->get_logger(), "收到深度图像 #%d, 时间戳: %ld.%09ld, 编码: %s", 
                       depth_count_, msg->header.stamp.sec, msg->header.stamp.nanosec, 
                       msg->encoding.c_str());
        } else {
            RCLCPP_DEBUG(this->get_logger(), "收到深度图像 #%d, 编码: %s", 
                        depth_count_, msg->encoding.c_str());
        }

        try {
            // 使用cv_bridge正确转换深度图像
            cv_bridge::CvImagePtr cv_ptr;
            if (msg->encoding == "32FC1") {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            } else if (msg->encoding == "16UC1") {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            } else {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
            }
            
            cv::Mat depth_image = cv_ptr->image;
            RCLCPP_DEBUG(this->get_logger(), "深度图像转换成功，尺寸: %dx%d, 类型: %d", 
                        depth_image.cols, depth_image.rows, depth_image.type());
            
            cv::Mat resized_image;
            cv::resize(depth_image, resized_image, cv::Size(1280, 720));
            
            // 将深度图像转换为伪彩色图像
            double minVal, maxVal;
            cv::minMaxLoc(resized_image, &minVal, &maxVal);
            RCLCPP_DEBUG(this->get_logger(), "深度值范围: min=%.3f, max=%.3f", minVal, maxVal);
            
            cv::Mat depth_colored;
            if (maxVal > minVal) {
                resized_image.convertTo(depth_colored, CV_8UC1, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));  
                cv::applyColorMap(depth_colored, depth_colored, cv::COLORMAP_JET);
                
                cv::imshow("Depth Image", depth_colored);
                cv::waitKey(1);
            } else {
                RCLCPP_WARN(this->get_logger(), "深度图像数据范围异常，跳过显示");
            }
            
        } catch (cv_bridge::Exception & e) {
            RCLCPP_ERROR(this->get_logger(), "深度图像转换失败: %s", e.what());
            return;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "深度图像处理异常: %s", e.what());
            return;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_depth;
    rclcpp::TimerBase::SharedPtr timer_;
    int color_count_;
    int depth_count_;
};

int main(int argc, char * argv[])
{
    std::cout << "启动VCF图像订阅者节点..." << std::endl;
    
    rclcpp::init(argc, argv);
    
    std::cout << "ROS 2初始化完成，创建节点..." << std::endl;
    
    auto node = std::make_shared<VCFSubscriber>();
    
    std::cout << "节点创建完成，开始运行..." << std::endl;
    std::cout << "按Ctrl+C退出程序" << std::endl;
    
    rclcpp::spin(node);
    
    std::cout << "节点已停止运行" << std::endl;
    
    rclcpp::shutdown();
    
    std::cout << "程序退出" << std::endl;
    
    return 0;
}