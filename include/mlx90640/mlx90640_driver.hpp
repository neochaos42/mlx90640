#ifndef MLX90640_DRIVER_HPP
#define MLX90640_DRIVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cstring>
#include "Adafruit_MLX90640.h"

class MLX90640Publisher : public rclcpp::Node
{
public:
    MLX90640Publisher();
    ~MLX90640Publisher();

private:
    void publishThermalImage();
    bool readTemperatureData(float *image);

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr thermal_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int i2c_fd_;
    Adafruit_MLX90640 mlx90640_;
};

#endif // MLX90640_DRIVER_HPP
