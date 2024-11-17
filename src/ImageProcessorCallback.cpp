#include "BabySharky.hpp"

// test function
void    AquabotNode::_imageProcessorCallback() {

    cv::Mat                 frame;
    cv::Mat                 gray;
    zbar::ImageScanner      scanner;

    this->_getImageData(frame);
    if (frame.empty())
        return ;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    zbar::Image             zbar_image(gray.cols, gray.rows, "Y800", gray.data, gray.total());

    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
    
    int n = scanner.scan(zbar_image);

    if (n > 0)
        for (auto symbol = zbar_image.symbol_begin(); symbol != zbar_image.symbol_end(); ++symbol)
            RCLCPP_INFO(this->get_logger(), "QR Code detected: %s", symbol->get_data().c_str());
    else
        RCLCPP_INFO(this->get_logger(), "No QR Code detected.");
}
