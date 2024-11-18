#include "BabySharky.hpp"

#define QR_DETECTOR 1
// #define OBSTACLE_DETECTOR 2

// test function
void    AquabotNode::_imageProcessorCallback() {

    cv::Mat                 frame;
    cv::Mat                 gray;

    int cameraMode = 0; // placeholder

    this->_getImageData(frame);
    if (frame.empty())
        return ;
    if (cameraMode == QR_DETECTOR) {

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        zbar::ImageScanner      scanner;
        zbar::Image             zbar_image(gray.cols, gray.rows, "Y800", gray.data, gray.total());

        scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

        int n = scanner.scan(zbar_image);

        if (n > 0)
            for (auto symbol = zbar_image.symbol_begin(); symbol != zbar_image.symbol_end(); ++symbol)
                RCLCPP_INFO(this->get_logger(), "QR Code detected: %s", symbol->get_data().c_str());
        else
            RCLCPP_INFO(this->get_logger(), "No QR Code detected.");
    }
    else {

        cv::Mat                             hsvImage;
        cv::Mat                             grayMask;
        std::vector<std::vector<cv::Point>> contours;

        cv::cvtColor(frame, hsvImage, cv::COLOR_BGR2HSV);
        cv::inRange(hsvImage, cv::Scalar(0, 0, 0), cv::Scalar(0, 0, 180), grayMask);
        cv::findContours(grayMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for (const auto &contour : contours)
            cv::drawContours(frame, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0, 255, 0), 2);

        // temp
        auto        msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        static auto publisher = \
            this->create_publisher<sensor_msgs::msg::Image> \
            ("/test", \
            rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable)
        );

        publisher->publish(*msg);
    }
}
