#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "LinuxHardwareSerial.hpp"
#include "SerialBridge.hpp"
#include "Button.hpp"

#define SERIAL_PORT "/dev/serial/by-path/pci-0000:00:14.0-usb-0:6:1.2"

using namespace std::chrono_literals;
using std::placeholders::_1;


class SerialBridgeClient : public rclcpp::Node {
public:
    SerialBridgeClient(const char port[], speed_t baud_rate = B9600)
            : Node("minecraft_robotics_client"), serial_dev(new LinuxHardwareSerial(port, baud_rate)),
              serial(new SerialBridge(serial_dev, 1024)) {
        //  add frame
        serial->add_frame(0, &button);

        //  setup subscription
        subscription_ = this->create_subscription<std_msgs::msg::Bool>(
                "/minecraft/toggle", 10, std::bind(&SerialBridgeClient::topic_callback, this, _1));
    }

private:

    void topic_callback(const std_msgs::msg::Bool &msg) {
        //  replace
        button.data.state = msg.data;

        //  send to serial
        serial->write(0);

        RCLCPP_DEBUG(this->get_logger(), "Send message to serial");
    }

    //  subscription
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;

    //  Serial Bridge
    SerialDev *serial_dev;
    SerialBridge *serial;

    //  Message
    Button button;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialBridgeClient>(SERIAL_PORT));
    rclcpp::shutdown();
    return 0;
}