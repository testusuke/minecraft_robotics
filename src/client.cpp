#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include "LinuxHardwareSerial.hpp"
#include "SerialBridge.hpp"
#include "Vector3.hpp"

#define SERIAL_PORT "/dev/serial/by-path/pci-0000:00:14.0-usb-0:6:1.2"

using namespace std::chrono_literals;
using std::placeholders::_1;


class SerialBridgeClient : public rclcpp::Node {
public:
    SerialBridgeClient(const char port[], speed_t baud_rate = B9600)
            : Node("serial_bridge_client"), serial_dev(new LinuxHardwareSerial(port, baud_rate)),
              serial(new SerialBridge(serial_dev, 1024)) {
        //  add frame
        serial->add_frame(0, &vector_msg[0]);
        serial->add_frame(1, &vector_msg[1]);

        //  setup subscription
        subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>(
                "serial_bridge_example_to_serial", 10, std::bind(&SerialBridgeClient::topic_callback, this, _1));

        //  setup publisher
        publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("serial_bridge_example_from_serial", 10);

        //  serial bridge watchdog
        timer_ = this->create_wall_timer(
                100ms, std::bind(&SerialBridgeClient::serial_callback, this));
    }

private:

    void serial_callback() {
        serial->update();

        if (vector_msg[1].was_updated()) {
            geometry_msgs::msg::Vector3 vector3;

            //  copy
            vector3.set__x(vector_msg[1].data.x);
            vector3.set__y(vector_msg[1].data.y);
            vector3.set__z(vector_msg[1].data.z);

            publisher_->publish(vector3);

            RCLCPP_DEBUG(this->get_logger(), "Received message from serial");
        }
    }

    void topic_callback(const geometry_msgs::msg::Vector3 &msg) {
        //  replace
        vector_msg[0].data.x = (float) msg.x;
        vector_msg[0].data.y = (float) msg.y;
        vector_msg[0].data.z = (float) msg.z;

        //  send to serial
        serial->write(0);

        RCLCPP_DEBUG(this->get_logger(), "Send message to serial");
    }

    //  publisher
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_;
    //  subscription
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscription_;

    rclcpp::TimerBase::SharedPtr timer_;

    //  Serial Bridge
    SerialDev *serial_dev;
    SerialBridge *serial;

    //  Message
    Vector3 vector_msg[2];
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialBridgeClient>(SERIAL_PORT));
    rclcpp::shutdown();
    return 0;
}