#include "BabySharky.hpp"

#define MAX_VIEW_DIST       800.0
#define HORIZON_LINE        255
#define X_RES               720
#define X_FOV               1.3962634

void    AquabotNode::_imageProcessorCallback() {

    int cameraState;

    this->_getCameraState(cameraState);

    if (!cameraState)
        return ;

    cv::Mat frame;

    this->_getImageData(frame);
    if (frame.empty())
        return ;
    if (cameraState == QR_DETECTOR) {

        std_msgs::msg::String   msg;
        cv::Mat                 gray;

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        zbar::ImageScanner  scanner;
        zbar::Image         zbar_image(gray.cols, gray.rows, "Y800", gray.data, gray.total());

        scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

        int n = scanner.scan(zbar_image);

        if (n > 0) {

            for (auto symbol = zbar_image.symbol_begin(); symbol != zbar_image.symbol_end(); ++symbol) {

                msg.data = symbol->get_data().c_str();
                this->_windTurbinCheckup->publish(msg);
                RCLCPP_INFO(this->get_logger(), "QR Code detected: %s", msg.data.c_str()); // debug
            }
        }
    }
    else if (cameraState == OBSTACLE_DETECTOR) {

        cv::Mat                             hsvImage;
        cv::Mat                             grayMask;
        std::vector<std::vector<cv::Point>> contours;
        double                              orientation[3];
        double                              dynamicHorizon;
        double                              finalResult[2] = {0.0, 0.0};
        static double                       yFov = 2 * std::atan((static_cast<double>(frame.rows) / frame.cols) * std::tan(X_FOV / 2));
        static double                       yPixelPerRadian = static_cast<double>(frame.rows) / yFov;
        static double                       xRadianPerPixel = X_FOV / frame.cols;
        static double                       xMid = frame.cols / 2;

        this->_getImuData(0, 0, orientation);
        dynamicHorizon = std::clamp(HORIZON_LINE - orientation[1] * yPixelPerRadian, 0.0, static_cast<double>(frame.rows));

        cv::cvtColor(frame, hsvImage, cv::COLOR_BGR2HSV);
        cv::inRange(hsvImage, cv::Scalar(0, 0, 0), cv::Scalar(0, 0, 180), grayMask);
        cv::findContours(grayMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for (const auto &contour : contours) {

            cv::Rect    boundingBox = cv::boundingRect(contour);
            int         yBoxMin = boundingBox.y + boundingBox.height;

            if (yBoxMin > dynamicHorizon) {

                double      distance = MAX_VIEW_DIST / (yBoxMin - dynamicHorizon + 1);
                double      deviation[2];

                deviation[LEFT] = (xMid - boundingBox.x) * xRadianPerPixel;
                deviation[RIGHT] = (boundingBox.x + boundingBox.width - xMid) * xRadianPerPixel;
                deviation[LEFT] = std::clamp(deviation[LEFT], 0.0, EPSILON * 45);
                deviation[RIGHT] = std::clamp(deviation[RIGHT], 0.0, EPSILON * 45);

                double result = deviation[deviation[LEFT] > deviation[RIGHT]];

                if (result < EPSILON)
                    result = 0.0;
                if (result && deviation[LEFT] < deviation[RIGHT])
                    result *= -1;
                if (result) {

                    finalResult[RANGE] = result;
                    finalResult[BEARING] = distance;
                }

                // debug
                if (result) {

                    cv::drawContours(frame, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0, 255, 0), 2);
                    cv::putText(frame, std::to_string(distance) + "m", cv::Point(boundingBox.x + boundingBox.width / 2, yBoxMin + 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1); // debug
                    cv::putText(frame, std::to_string(result) + "rad", cv::Point(boundingBox.x + boundingBox.width / 2, yBoxMin + 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1); // debug
                }
            }
        }
        this->_setAvoidanceTarget(finalResult);
    }

    // debug image topic
    static auto publisher = this->create_publisher<sensor_msgs::msg::Image>("/test", rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable));
    auto        msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

    publisher->publish(*msg);
}
