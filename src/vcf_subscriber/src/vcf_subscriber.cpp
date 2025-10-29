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

        // 使用默认的QoS配置
        auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

        // 创建图像订阅者，订阅名为"vcf_image"的主题
        subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/image_raw/compressed", default_qos,
            std::bind(&VCFSubscriber::image_callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "已创建彩色图像订阅者，主题: /image_raw/compressed");

        subscription_depth = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw/compressed/depth", default_qos,
            std::bind(&VCFSubscriber::image_callback2, this, _1));

        RCLCPP_INFO(this->get_logger(), "已创建深度图像订阅者，主题: /image_raw/compressed/depth");
        RCLCPP_INFO(this->get_logger(), "节点初始化完成，等待图像数据...");

        // 添加定时器，定期输出节点状态
        timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&VCFSubscriber::timer_callback, this));

        color_count_ = 0;
        depth_count_ = 0;

        // 创建OpenCV窗口
        cv::namedWindow("VCF Image", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("Depth Image", cv::WINDOW_AUTOSIZE);
    }

    ~VCFSubscriber()
    {
        // 关闭OpenCV窗口
        cv::destroyAllWindows();
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

        // 检查数据是否为空
        if (msg->data.empty())
        {
            RCLCPP_WARN(this->get_logger(), "收到空的彩色图像数据 #%d", color_count_);
            return;
        }

        if (color_count_ % 30 == 0)
        {
            RCLCPP_INFO(this->get_logger(), "收到彩色图像 #%d, 时间戳: %d.%09u, 数据大小: %zu bytes",
                        color_count_, msg->header.stamp.sec, msg->header.stamp.nanosec, msg->data.size());
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(), "收到彩色图像 #%d, 大小: %zu bytes", color_count_, msg->data.size());
        }

        try
        {
            // 直接使用OpenCV解码压缩图像
            cv::Mat compressed_data(1, msg->data.size(), CV_8UC1, (void *)msg->data.data());
            cv::Mat decoded_image = cv::imdecode(compressed_data, cv::IMREAD_COLOR);

            if (decoded_image.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "图像解码失败，可能是不支持的格式");
                return;
            }

            RCLCPP_DEBUG(this->get_logger(), "彩色图像解码成功，尺寸: %dx%d",
                         decoded_image.cols, decoded_image.rows);

            // 调整尺寸并显示
            cv::Mat resized_image;
            cv::resize(decoded_image, resized_image, cv::Size(1280, 720));

            // 注意：OpenCV默认使用BGR格式，但imdecode会根据数据自动处理
            // 如果颜色不对，可以尝试转换
            // cv::cvtColor(resized_image, resized_image, cv::COLOR_BGR2RGB);

            RCLCPP_DEBUG(this->get_logger(), "彩色图像已调整尺寸: 1280x720");

            cv::imshow("VCF Image", resized_image);
            int key = cv::waitKey(1);
            if (key == 27)
            { // ESC键
                RCLCPP_INFO(this->get_logger(), "用户按下ESC键，准备退出...");
                rclcpp::shutdown();
            }
        }
        catch (const cv::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "OpenCV处理异常: %s", e.what());
            return;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "彩色图像处理异常: %s", e.what());
            return;
        }
    }

    void image_callback2(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        depth_count_++;

        // // 检查数据是否为空
        // if (msg->data.empty())
        // {
        //     RCLCPP_WARN(this->get_logger(), "收到空的深度图像数据 #%d", depth_count_);
        //     return;
        // }

        // // 验证图像尺寸和数据的完整性
        // size_t expected_size = msg->height * msg->step;
        // if (msg->data.size() != expected_size)
        // {
        //     RCLCPP_ERROR(this->get_logger(),
        //                  "深度图像数据大小不匹配! 期望: %zu (height=%u * step=%u), 实际: %zu",
        //                  expected_size, msg->height, msg->step, msg->data.size());

        //     // 尝试计算正确的步长
        //     if (msg->height > 0 && msg->width > 0)
        //     {
        //         size_t calculated_step = msg->data.size() / msg->height;
        //         RCLCPP_INFO(this->get_logger(),
        //                     "建议步长: %zu (数据大小 %zu / 高度 %u)",
        //                     calculated_step, msg->data.size(), msg->height);
        //     }
        //     return;
        // }

        if (depth_count_ % 30 == 0)
        {
            // depth camera resolution is: 640x480, datasize is: 640*480*4=1228800
            // e.g.: 收到深度图像 #2670, 时间戳: 1761718899.898250000, 编码: 32FC1, 尺寸: 0x0, 步长: 0, 数据大小: 1228800
            RCLCPP_INFO(this->get_logger(),
                        "收到深度图像 #%d, 时间戳: %d.%09u, 编码: %s, 尺寸: %dx%d, 步长: %u, 数据大小: %zu",
                        depth_count_, msg->header.stamp.sec, msg->header.stamp.nanosec,
                        msg->encoding.c_str(), msg->width, msg->height, msg->step, msg->data.size());
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(),
                         "收到深度图像 #%d, 编码: %s, 尺寸: %dx%d",
                         depth_count_, msg->encoding.c_str(), msg->width, msg->height);
        }

        // try
        // {
        //     // 创建新的Image消息来修复步长问题
        //     auto fixed_msg = std::make_shared<sensor_msgs::msg::Image>();
        //     *fixed_msg = *msg; // 复制原消息

        //     // 重新计算正确的步长
        //     if (msg->encoding == "32FC1")
        //     {
        //         fixed_msg->step = msg->width * 4; // 32位浮点数 = 4字节
        //     }
        //     else if (msg->encoding == "16UC1")
        //     {
        //         fixed_msg->step = msg->width * 2; // 16位无符号整数 = 2字节
        //     }
        //     else if (msg->encoding == "8UC1" || msg->encoding == "mono8")
        //     {
        //         fixed_msg->step = msg->width * 1; // 8位 = 1字节
        //     }
        //     else
        //     {
        //         // 对于未知编码，尝试自动计算
        //         if (msg->height > 0)
        //         {
        //             fixed_msg->step = msg->data.size() / msg->height;
        //             RCLCPP_WARN(this->get_logger(),
        //                         "未知编码 %s，计算步长为: %u",
        //                         msg->encoding.c_str(), fixed_msg->step);
        //         }
        //     }

        //     RCLCPP_DEBUG(this->get_logger(),
        //                  "修复后的步长: %u (原步长: %u)",
        //                  fixed_msg->step, msg->step);

        //     // 使用cv_bridge转换深度图像
        //     cv_bridge::CvImagePtr cv_ptr;

        //     if (msg->encoding == "32FC1")
        //     {
        //         cv_ptr = cv_bridge::toCvCopy(fixed_msg, sensor_msgs::image_encodings::TYPE_32FC1);
        //     }
        //     else if (msg->encoding == "16UC1")
        //     {
        //         cv_ptr = cv_bridge::toCvCopy(fixed_msg, sensor_msgs::image_encodings::TYPE_16UC1);
        //     }
        //     else if (msg->encoding == "mono8" || msg->encoding == "8UC1")
        //     {
        //         cv_ptr = cv_bridge::toCvCopy(fixed_msg, sensor_msgs::image_encodings::MONO8);
        //     }
        //     else
        //     {
        //         RCLCPP_WARN(this->get_logger(),
        //                     "不支持的深度图像编码: %s，尝试自动处理",
        //                     msg->encoding.c_str());
        //         cv_ptr = cv_bridge::toCvCopy(fixed_msg); // 让cv_bridge自动处理
        //     }

        //     cv::Mat depth_image = cv_ptr->image;

        //     if (depth_image.empty())
        //     {
        //         RCLCPP_ERROR(this->get_logger(), "深度图像转换后为空");
        //         return;
        //     }

        //     RCLCPP_DEBUG(this->get_logger(),
        //                  "深度图像转换成功，尺寸: %dx%d, 类型: %d, 通道: %d",
        //                  depth_image.cols, depth_image.rows, depth_image.type(), depth_image.channels());

        //     // 调整尺寸
        //     cv::Mat resized_image;
        //     cv::resize(depth_image, resized_image, cv::Size(1280, 720));

        //     // 将深度图像转换为伪彩色图像
        //     double minVal, maxVal;
        //     cv::minMaxLoc(resized_image, &minVal, &maxVal);
        //     RCLCPP_DEBUG(this->get_logger(),
        //                  "深度值范围: min=%.3f, max=%.3f", minVal, maxVal);

        //     cv::Mat depth_colored;
        //     if (maxVal > minVal && std::isfinite(minVal) && std::isfinite(maxVal))
        //     {
        //         // 归一化到0-255
        //         cv::Mat normalized;
        //         resized_image.convertTo(normalized, CV_8UC1, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));

        //         // 应用伪彩色
        //         cv::applyColorMap(normalized, depth_colored, cv::COLORMAP_JET);

        //         cv::imshow("Depth Image", depth_colored);
        //         cv::waitKey(1);
        //     }
        //     else
        //     {
        //         RCLCPP_WARN(this->get_logger(),
        //                     "深度图像数据范围异常 (min=%.3f, max=%.3f) 或包含非有限值，跳过显示",
        //                     minVal, maxVal);

        //         // 显示原始图像作为备选
        //         cv::Mat display_image;
        //         if (resized_image.type() == CV_32FC1)
        //         {
        //             // 对于32FC1，直接缩放显示
        //             resized_image.convertTo(display_image, CV_8UC1, 1.0 / 1000.0 * 255); // 假设单位是毫米
        //         }
        //         else
        //         {
        //             resized_image.convertTo(display_image, CV_8UC1);
        //         }
        //         cv::imshow("Depth Image", display_image);
        //         cv::waitKey(1);
        //     }
        // }
        // catch (const cv_bridge::Exception &e)
        // {
        //     RCLCPP_ERROR(this->get_logger(), "深度图像转换失败: %s", e.what());

        //     // 尝试直接处理原始数据
        //     try
        //     {
        //         RCLCPP_INFO(this->get_logger(), "尝试直接处理原始数据...");
        //         process_raw_depth_data(msg);
        //     }
        //     catch (const std::exception &e2)
        //     {
        //         RCLCPP_ERROR(this->get_logger(), "原始数据处理也失败: %s", e2.what());
        //     }
        //     return;
        // }
        // catch (const std::exception &e)
        // {
        //     RCLCPP_ERROR(this->get_logger(), "深度图像处理异常: %s", e.what());
        //     return;
        // }

        cv::Mat depth_image;
        depth_image = cv::Mat(480, 640, CV_32FC1, const_cast<uchar *>(msg->data.data())); // 深度图像为32FC1格式

        cv::Mat resized_image;
        cv::resize(depth_image, resized_image, cv::Size(1280, 720)); // 调整尺寸
        // 将深度图像转换为伪彩色图像
        double minVal, maxVal;
        cv::minMaxLoc(resized_image, &minVal, &maxVal); // 获取最小和最大深度值
        cv::Mat depth_colored;
        resized_image.convertTo(depth_colored, CV_8UC1, 255 / (maxVal - minVal), -minVal * 255 / (maxVal - minVal)); // 归一化到0-255
        cv::applyColorMap(depth_colored, depth_colored, cv::COLORMAP_JET); // 应用伪彩色
        cv::imshow("Depth Image", depth_colored);
        cv::waitKey(1); // 等待1毫秒以更新窗口
    }

    // 辅助函数：直接处理原始深度数据
    void process_raw_depth_data(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // 根据编码类型直接解析数据
        if (msg->encoding == "16UC1")
        {
            // 16位无符号整数深度图
            cv::Mat depth_image(msg->height, msg->width, CV_16UC1, (void *)msg->data.data());
            if (!depth_image.empty())
            {
                cv::Mat resized;
                cv::resize(depth_image, resized, cv::Size(1280, 720));
                cv::imshow("Depth Image Raw", resized);
                cv::waitKey(1);
            }
        }
        else if (msg->encoding == "32FC1")
        {
            // 32位浮点数深度图
            cv::Mat depth_image(msg->height, msg->width, CV_32FC1, (void *)msg->data.data());
            if (!depth_image.empty())
            {
                cv::Mat resized;
                cv::resize(depth_image, resized, cv::Size(1280, 720));

                // 转换为可显示的格式
                cv::Mat display;
                resized.convertTo(display, CV_8UC1, 255.0 / 10.0); // 假设10米最大范围
                cv::imshow("Depth Image Raw", display);
                cv::waitKey(1);
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_depth;
    rclcpp::TimerBase::SharedPtr timer_;
    int color_count_;
    int depth_count_;
};

int main(int argc, char *argv[])
{
    std::cout << "启动VCF图像订阅者节点..." << std::endl;

    rclcpp::init(argc, argv);

    std::cout << "ROS 2初始化完成，创建节点..." << std::endl;

    auto node = std::make_shared<VCFSubscriber>();

    std::cout << "节点创建完成，开始运行..." << std::endl;
    std::cout << "按ESC键退出程序" << std::endl;

    try
    {
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        std::cerr << "节点运行异常: " << e.what() << std::endl;
    }

    std::cout << "节点已停止运行" << std::endl;

    rclcpp::shutdown();

    std::cout << "程序退出" << std::endl;

    return 0;
}