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

/// Global flag used to alternate the X-position in the command message
bool flip = false;

/**
 * @class ROSDDSBridge
 * @brief A ROS 2 node that periodically publishes `arm_msgs::ArmCommand` messages to a DDS topic and logs the operation to a ROS topic.
 */
class ROSDDSBridge : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for the ROSDDSBridge node.
   * Initializes DDS entities (participant, topic, publisher, writer),
   * sets QoS policies, and creates a periodic ROS timer to invoke the DDS callback.
   */
  ROSDDSBridge()
      : Node("ros_dds_bridge"),
        participant_(0),
        dds_topic_(participant_, "ArmCommandTopic"),
        dds_publisher_(participant_),
        dds_qos_(
            // Start from the publisher’s default DataWriter QoS
            dds_publisher_.default_datawriter_qos()
            // Set reliable delivery
            << dds::core::policy::Reliability::Reliable()
            // Optional history: << dds::core::policy::History::KeepLast(10)
            << dds::core::policy::Durability::Volatile()),
        dds_writer_(dds_publisher_, dds_topic_, dds_qos_)
  {
    // Create a ROS publisher for logging DDS write status
    publisher_ = this->create_publisher<std_msgs::msg::String>("dds_status", 10);

    // Set up a timer to publish to DDS every 500ms
    timer_ = this->create_wall_timer(
        500ms, std::bind(&ROSDDSBridge::dds_callback, this));
  }

private:
  /**
   * @brief Callback function invoked periodically by the ROS timer.
   * Publishes a sample `ArmCommand` message to the DDS topic and logs the status to a ROS topic.
   */
  void dds_callback()
  {
    bool success_flag = false;

    RCLCPP_INFO(this->get_logger(), "=== [DDS PUB] Waiting for IK SUB.");

    // Wait until at least one subscriber is matched
    while (dds_writer_.publication_matched_status().current_count() == 0)
      std::this_thread::sleep_for(std::chrono::milliseconds(20));

    RCLCPP_INFO(this->get_logger(), "SUB FOUND");

    // Create timestamp for the message header
    uint64_t total_ns = now().nanoseconds();

    // Construct ArmCommand message with default pose
    arm_msgs::ArmCommand command(
        arm_msgs::Header(
            arm_msgs::Time((total_ns / 1'000'000'000ULL), (total_ns % 1'000'000'000ULL)),
            "end_effector_frame"),
        arm_msgs::Pose(
            arm_msgs::Point(0.0, 0.0, 0.0),
            arm_msgs::Quaternion(0.0, 0.0, 0.0, 1.0)));

    // Flip x position between 0.0 and 1.0 on every call
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
      // Log and publish to DDS
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
      // Log DDS write error
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "=== [DDS PUB] Exception: " << e.what());
    }

    // Construct a ROS log message indicating DDS status
    auto message = std_msgs::msg::String();

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

    message.data = ss.str();

    // Publish the ROS log message
    publisher_->publish(message);
  }

  /// ROS timer for triggering periodic DDS writes
  rclcpp::TimerBase::SharedPtr timer_;

  /// ROS publisher for logging DDS publish status
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  /// DDS DomainParticipant
  dds::domain::DomainParticipant participant_;

  /// DDS Topic for ArmCommand messages
  dds::topic::Topic<arm_msgs::ArmCommand> dds_topic_;

  /// DDS Publisher
  dds::pub::Publisher dds_publisher_;

  /// DDS QoS settings for the DataWriter
  dds::pub::qos::DataWriterQos dds_qos_;

  /// DDS DataWriter for ArmCommand messages
  dds::pub::DataWriter<arm_msgs::ArmCommand> dds_writer_;
};

/**
 * @brief Main entry point for the ROSDDSBridge node.
 */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ROSDDSBridge>());
  rclcpp::shutdown();
  return 0;
}
