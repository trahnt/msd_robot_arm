#include "sauron/rviz2_panel.hpp"

namespace custom_panel {
  RvizPushButtonPanel::RvizPushButtonPanel(QWidget *parent)
    : Panel{parent}
    , ui_(std::make_unique<Ui::gui>())
    , node_{nullptr}
  {
    // Extend the widget with all attributes and children from UI file
    ui_->setupUi(this);

    // Init rclcpp node
    auto options = rclcpp::NodeOptions().arguments(
        {"--ros-args", "--remap", "__node:=rviz_push_button_node", "--"});
    node_ = std::make_shared<rclcpp::Node>("_", options);

    // MAKE PUBLISHER CORRESPONDING TO THE BUTTON
    button1_pub_ = node_->create_publisher<std_msgs::msg::Bool>("button1Publisher", 1);

    // Prepare msg
    msg_.data = true;
  }



  RvizPushButtonPanel::~RvizPushButtonPanel() {}



  void RvizPushButtonPanel::load(const rviz_common::Config &config) {
    Panel::load(config);
    if (auto push_button_config = config.mapGetChild({"PushButton"}); push_button_config.isValid()) {

      // I don't care if it was pressed honestly
      // if (QVariant count_button_1 {0}; push_button_config.mapGetValue({"count_button_1"}, &count_button_1)) {
        // RCLCPP_INFO_STREAM(node_->get_logger(), "Button 1 was pressed ");
      // }
    }
  }
  


  void RvizPushButtonPanel::save(rviz_common::Config config) const {
    Panel::save(config);
    rviz_common::Config push_button_config = config.mapMakeChild({"PushButton"});
    // push_button_config.mapSetValue({"count_button_1"}, {count_button_1_});
  }
  


  void RvizPushButtonPanel::on_pushButton1_clicked() {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Published to button1 topic!");
    button1_pub_->publish(msg_);
    // When clicked, set the text to say it was clicked and at what time
    std::time_t now = std::time(nullptr); 
    char timeCString[16];  // worst case string is XX:XX:XX\n, so 16 should be plenty
    std::strftime(timeCString, sizeof(timeCString), "%H:%M:%S", std::localtime(&now));
    ui_->label_1->setText(QString::fromStdString({std::string{"Button 1 was clicked at "} + std::string(timeCString)}));
  }

} // custom_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(custom_panel::RvizPushButtonPanel, rviz_common::Panel)
