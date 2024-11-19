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
        double                              orientation[3];
        double                              dynamicHorizon;
        double                              vfov = 2 * std::atan((static_cast<double>(frame.rows) / frame.cols) * std::tan(1.3962634 / 2));

        this->_getImuData(0, 0, orientation);
        dynamicHorizon = std::clamp(254 - orientation[1] * (frame.rows / vfov), 0.0, static_cast<double>(frame.rows));

        cv::cvtColor(frame, hsvImage, cv::COLOR_BGR2HSV);
        cv::inRange(hsvImage, cv::Scalar(0, 0, 0), cv::Scalar(0, 0, 180), grayMask);
        cv::findContours(grayMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for (const auto &contour : contours) {

            // cv::drawContours(frame, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0, 255, 0), 2); // debug

            cv::Rect    boundingBox = cv::boundingRect(contour);
            double      yMin = boundingBox.y + boundingBox.height;
            double      xMid = boundingBox.x + (double)boundingBox.width / 2;

            if (yMin > dynamicHorizon) {

                double      distance = 800.0 / (yMin - dynamicHorizon + 1);
                std::string distanceText;
                std::string test;

                distanceText = std::to_string(distance) + "m"; // debug
                if (distance) {

                    if (xMid > 360)
                        test = "left " + std::to_string(boundingBox.x) + "p"; // debug
                    else
                        test = "right " + std::to_string(boundingBox.x + boundingBox.width) + "p"; // debug
                    cv::putText(frame, test, cv::Point(static_cast<int>(xMid), static_cast<int>(yMin) + 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1); // debug
                }
                cv::putText(frame, distanceText, cv::Point(static_cast<int>(xMid), static_cast<int>(yMin) + 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);

                // double range[2];
                // this->_getCriticalWindTurbinData(range);
                // std::cout << std::to_string(range[RANGE]) + "m de range pour" + std::to_string(distance) + "m de speculation\n";
            }
        }

        // debug
        // cv::line(frame, cv::Point(0, static_cast<int>(dynamicHorizon)), cv::Point(frame.cols, static_cast<int>(dynamicHorizon)), cv::Scalar(0, 0, 255), 1);
        // cv::putText(frame, "Horizon line", cv::Point(10, static_cast<int>(dynamicHorizon) - 10), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 1);
    }
    // debug image topic
    auto        msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    static auto publisher = this->create_publisher<sensor_msgs::msg::Image>("/test", rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable));

    publisher->publish(*msg);
}
