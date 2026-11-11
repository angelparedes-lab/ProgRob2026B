#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>

using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;

static cv::Mat annotate(const cv::Mat & frame, const std::vector<std::string> & lines)
{
    cv::Mat out = frame.clone();
    const int cx = out.cols/2;
    const int cy = out.rows/2;

    cv::line(out,{cx - 15,cy},{cx + 15,cy},{255, 255, 255},1);
    cv::line(out,{cx,cy - 15},{cx,cy + 15},{255, 255, 255},1);
    cv::circle(out,{cx,cy}, 4, {0, 255, 255}, 1);
    for (size_t i = 0; i < lines.size(); ++i)
    {
        const cv::Point org(10,22 + 20 * static_cast<int>(i));
        cv::putText(out, lines[i], org, cv::FONT_HERSHEY_SIMPLEX, 0.55, {0, 0, 0}, 3, cv::LINE_AA);
        cv::putText(out, lines[i], org, cv::FONT_HERSHEY_SIMPLEX, 0.55, {255, 255, 255}, 1, cv::LINE_AA);
    }
    return out;
}

class CameraViewer : public rclcpp::Node
{
public:
    CameraViewer() : Node("camera_viewer")
    {
        const auto image_topic = declare_parameter<std::string>("image_topic", "/camera/image_raw");
        const auto info_topic = declare_parameter<std::string>("info_topic", "/camera/camera_info");
        const auto debug_topic = declare_parameter<std::string>("debug_topic", "/camera/image_debug");

        snapshot_path_ = declare_parameter<std::string>("snapshot_path", "");

        sub_image_ = create_subscription<Image>(image_topic, rclcpp::SensorDataQoS(),
            std::bind(&CameraViewer::on_image, this, std::placeholders::_1));
        sub_info_ = create_subscription<CameraInfo>(info_topic,10,
            std::bind(&CameraViewer::on_info, this, std::placeholders::_1));
        pub_debug_ = create_publisher<Image>(debug_topic, 10);

        t_last_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(get_logger(),"Esperando imagenes en %s y %s ...", 
            image_topic.c_str(), info_topic.c_str());
    }
private:
    void on_info(const CameraInfo::SharedPtr msg)
    {
        if (info_logged_) {return;}
        info_logged_ = true;
        RCLCPP_INFO(get_logger(), 
            "camera_info: %ux%u, frame_id=\"%s\"\n fx=%.1f fy=%.1f cx=%1.f cy=%.1f",
            msg->width, msg->height, msg->header.frame_id.c_str(),
            msg->k[0], msg->k[4], msg->k[2], msg->k[5]);
    }

    void on_image(const Image::ConstSharedPtr msg)
    {
        if (!frame_id_logged_)
        {
            frame_id_logged_ = true;
            RCLCPP_INFO(get_logger(), "image: %ux%u, encoding=%s, frame_id=\"%s\"", 
                msg->width, msg->height, msg->encoding.c_str(), msg->header.frame_id.c_str());
        }

        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception & e)
        {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        const cv::Mat frame = cv_ptr->image;

        ++frames_;
        if (frames_ % 15 == 0)
        {
            const auto now = std::chrono::steady_clock::now();
            fps_ = 15.0 / std::chrono::duration<double>(now - t_last_).count();
            t_last_ = now;
        }

        const cv::Vec3b px = frame.at<cv::Vec3b>(frame.rows/2, frame.cols/2);
        char l1[64], l2[64];
        std::snprintf(l1, sizeof(l1), "%dx%d %4.1f FPS", frame.cols, frame.rows, fps_);
        std::snprintf(l2, sizeof(l2), "Pixel BGR=: (%d, %d, %d)", px[0], px[1], px[2]);
        const cv::Mat debug = annotate(frame, {l1, l2});

        pub_debug_->publish(*cv_bridge::CvImage(msg->header, "bgr8", debug).toImageMsg());

        if (!snapshot_path_.empty() && frames_ == 10)
        {
            cv::imwrite(snapshot_path_, frame);
            RCLCPP_INFO(get_logger(), "Snapshot saved to %s", snapshot_path_.c_str());
        }
    }

    rclcpp::Subscription<Image>::SharedPtr sub_image_;
    rclcpp::Subscription<CameraInfo>::SharedPtr sub_info_;
    rclcpp::Publisher<Image>::SharedPtr pub_debug_;
    std::string snapshot_path_;
    std::chrono::steady_clock::time_point t_last_;
    double fps_ = {0.0};
    int frames_ = {0};
    bool info_logged_ = {false};
    bool frame_id_logged_ = {false};
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraViewer>());
    rclcpp::shutdown();
    return 0;
}
