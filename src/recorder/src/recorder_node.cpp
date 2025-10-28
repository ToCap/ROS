#include "mcap_logger/mcap_logger_node.hpp"
#include <rclcpp/serialization.hpp>

using namespace std::chrono_literals;

namespace recorder
{

McapLoggerNode::McapLoggerNode() 
: Node("mcap_logger_node")
{
  // Options d’écriture MCAP
  mcap::McapWriterOptions opts("ros2_mcap_example");
  opts.compression = mcap::Compression::None;
  writer_.open("output.mcap", opts);

  // Définir le schéma ROS 2
  mcap::Schema schema;
  schema.name = "std_msgs/msg/String";
  schema.encoding = "ros2msg";
  schema.data = "string data";
  schema_id_ = writer_.addSchema(schema);

  // Créer le canal (topic)
  mcap::Channel channel;
  channel.topic = "/chatter";
  channel.messageEncoding = "cdr";  // Encodage ROS 2
  channel.schemaId = schema_id_;
  channel_id_ = writer_.addChannel(channel);

  // Souscrire au topic
  sub_ = this->create_subscription<std_msgs::msg::String>(
    "/chatter", 10,
    std::bind(&McapLoggerNode::callback, this, std::placeholders::_1)
  );

  RCLCPP_INFO(this->get_logger(), "MCAP Logger node started, recording /chatter");
}

McapLoggerNode::~McapLoggerNode()
{
  writer_.close();
  RCLCPP_INFO(this->get_logger(), "MCAP file closed.");
}

void McapLoggerNode::callback(const std_msgs::msg::String::SharedPtr msg)
{
  // Sérialisation du message ROS 2
  rclcpp::SerializedMessage serialized;
  rclcpp::Serialization<std_msgs::msg::String> serializer;
  serializer.serialize_message(msg.get(), &serialized);

  // Création du message MCAP
  mcap::Message m;
  m.channelId = channel_id_;
  m.sequence = ++sequence_;
  m.publishTime = this->get_clock()->now().nanoseconds();
  m.data = reinterpret_cast<const char*>(serialized.get_rcl_serialized_message().buffer);
  m.dataSize = serialized.size();

  writer_.write(m);

  RCLCPP_INFO(this->get_logger(), "Logged message to MCAP: %s", msg->data.c_str());
}

}