#include <cstdio>
#include <iostream>
#include "arm_msgs.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <dds/dds.hpp>
#include <chrono>
#include <thread>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;
bool flip = false;

class ROSDDSBridge : public rclcpp::Node
{
public:
  ROSDDSBridge()
      : Node("ros_dds_bridge"),
        participant_(0),
        dds_topic_(participant_, "ArmCommandTopic"),
        dds_publisher_(participant_),
        dds_qos_(
            // start from the publisher’s default DataWriter QoS
            dds_publisher_.default_datawriter_qos()
            // then tweak reliability, history, durability, etc
            << dds::core::policy::Reliability::Reliable()
            // << dds::core::policy::History::KeepLast(10)
            << dds::core::policy::Durability::Volatile()),
        dds_writer_(dds_publisher_, dds_topic_, dds_qos_)

  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("dds_status", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&ROSDDSBridge::dds_callback, this));
  }

private:
  void dds_callback()
  // (double xyzq[7])
  {
    bool success_flag = false;
    RCLCPP_INFO(this->get_logger(), "=== [DDS PUB] Waiting for IK SUB.");
    while (dds_writer_.publication_matched_status().current_count() == 0)
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    RCLCPP_INFO(this->get_logger(), "SUB FOUND");
    uint64_t total_ns = now().nanoseconds();
    arm_msgs::ArmCommand command(
        arm_msgs::Header(
            arm_msgs::Time((total_ns / 1'000'000'000ULL), (total_ns % 1'000'000'000ULL)),
            "end_effector_frame"),
        arm_msgs::Pose(
            arm_msgs::Point(0.0, 0.0, 0.0),
            arm_msgs::Quaternion(0.0, 0.0, 0.0, 1.0)));
    if (flip)
    {
      command.pose().position().x() = 1.0;
      flip = !flip;
    }
    else
    {
      flip = !flip;
    }
    try
    {
      RCLCPP_INFO(this->get_logger(),
                  "=== [DDS PUB] Write sample XYZQuat: [%f, %f, %f, %f, %f, %f, %f]",
                  command.pose().position().x(),
                  command.pose().position().y(),
                  command.pose().position().z(),
                  command.pose().orientation().x(),
                  command.pose().orientation().y(),
                  command.pose().orientation().z(),
                  command.pose().orientation().w());
      dds_writer_.write(command);
      RCLCPP_INFO(this->get_logger(), "=== [DDS PUB] Write Sample Successful.");
      success_flag = true;
    }
    catch (const dds::core::Exception &e)
    {
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "=== [DDS PUB] Exception: " << e.what());
    }

    auto message = std_msgs::msg::String();

    // build a single string via ostringstream
    std::ostringstream ss;
    ss << "DDS PUB "
       << (success_flag ? "SUCCESS" : "FAILURE")
       << "  XYZQuat: ["
       << command.pose().position().x() << ", "
       << command.pose().position().y() << ", "
       << command.pose().position().z() << ","
       << command.pose().orientation().x() << ", "
       << command.pose().orientation().y() << ", "
       << command.pose().orientation().z() << ", "
       << command.pose().orientation().w() << "]";

    // grab the composed string once
    message.data = ss.str();
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  dds::domain::DomainParticipant participant_;
  dds::topic::Topic<arm_msgs::ArmCommand> dds_topic_;
  dds::pub::Publisher dds_publisher_;
  dds::pub::qos::DataWriterQos dds_qos_;
  dds::pub::DataWriter<arm_msgs::ArmCommand> dds_writer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ROSDDSBridge>());
  rclcpp::shutdown();
  return 0;
}