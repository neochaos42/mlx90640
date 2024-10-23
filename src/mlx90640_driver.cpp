#include "mlx90640_driver.hpp"

MLX90640Publisher::MLX90640Publisher() : Node("mlx90640_publisher")
{
    // Set up the publisher for the thermal image
    thermal_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/thermal_image", 10);

    // Open I2C device
    i2c_fd_ = open("/dev/i2c-1", O_RDWR);
    if (i2c_fd_ < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open I2C bus");
        rclcpp::shutdown();
    }

    // Set the I2C address for MLX90640
    if (ioctl(i2c_fd_, I2C_SLAVE, 0x33) < 0) // 0x33 is the default address of MLX90640
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to set I2C address");
        rclcpp::shutdown();
    }

    // Initialize the MLX90640
    if (!mlx90640_.begin(0x33))
    {
        RCLCPP_ERROR(this->get_logger(), "Could not find MLX90640 sensor!");
        rclcpp::shutdown();
    }

    // Set the refresh rate (32Hz in this case)
    mlx90640_.setMode(MLX90640_CHESS);
    mlx90640_.setRefreshRate(MLX90640_32_HZ);

    // Timer to periodically publish the thermal image
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&MLX90640Publisher::publishThermalImage, this));
}

MLX90640Publisher::~MLX90640Publisher()
{
    if (i2c_fd_ > 0)
    {
        close(i2c_fd_);
    }
}

void MLX90640Publisher::publishThermalImage()
{
    float image[32 * 24]; // MLX90640 has a resolution of 32x24

    // Read temperature data from the sensor
    if (readTemperatureData(image))
    {
        // Create the ROS2 Image message
        auto msg = sensor_msgs::msg::Image();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "thermal_frame";
        msg.height = 24;
        msg.width = 32;
        msg.encoding = "mono16"; // 16-bit grayscale for temperature data
        msg.is_bigendian = false;
        msg.step = 32 * sizeof(float);
        msg.data.resize(32 * 24 * sizeof(float));

        // Copy the temperature data into the ROS2 message
        memcpy(msg.data.data(), image, 32 * 24 * sizeof(float));

        // Publish the message
        thermal_pub_->publish(msg);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to read temperature data");
    }
}

bool MLX90640Publisher::readTemperatureData(float *image)
{
    uint16_t frameData[834];
    float emissivity = 1.0;
    float mlx90640To[32 * 24]; // Temperature data array

    // Read the frame data from the sensor
    int status = mlx90640_.getFrameData(frameData);
    if (status < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to read frame data from MLX90640");
        return false;
    }

    // Process the temperature data
    mlx90640_.calculateTo(frameData, emissivity, mlx90640To);

    // Copy the processed temperature data into the provided array
    memcpy(image, mlx90640To, 32 * 24 * sizeof(float));

    return true;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MLX90640Publisher>());
    rclcpp::shutdown();
    return 0;
}
