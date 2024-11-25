#include "sauron/rviz2_panel.hpp"

namespace custom_panel {
  SauronPanel::SauronPanel(QWidget *parent)
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

    // MAKE PUBLISHERS CORRESPONDING TO THE BUTTON
    // For now, sends a bool true message to the to the home_request topic
    homerPub_ = node_->create_publisher<std_msgs::msg::Bool>("home_request", 1);

    // Prepare msg
    msg_.data = true;
  }



  SauronPanel::~SauronPanel() {}



  void SauronPanel::load(const rviz_common::Config &config) {
    Panel::load(config);

    // I don't care if it was pressed honestly
    // if (auto push_button_config = config.mapGetChild({"HomeButton"}); push_button_config.isValid()) {

      // I don't care if it was pressed honestly
      // if (QVariant count_button_1 {0}; push_button_config.mapGetValue({"count_button_1"}, &count_button_1)) {
        // RCLCPP_INFO_STREAM(node_->get_logger(), "Button 1 was pressed ");
      // }
    // }
  }
  


  void SauronPanel::save(rviz_common::Config config) const {
    Panel::save(config);
    // rviz_common::Config push_button_config = config.mapMakeChild({"PushButton"});
    // push_button_config.mapSetValue({"count_button_1"}, {count_button_1_});
  }
  


  void SauronPanel::on_homeButton_clicked() {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Home message sent!");
    homerPub_->publish(msg_);
    // When clicked, set the text to say it was clicked and at what time
    std::time_t now = std::time(nullptr); 
    char timeCString[16];  // worst case string is XX:XX:XX\n, so 16 should be plenty
    std::strftime(timeCString, sizeof(timeCString), "%H:%M:%S", std::localtime(&now));
    ui_->homeLabel->setText(QString::fromStdString({std::string{"Homed at "} + std::string(timeCString)}));
  }

} // custom_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(custom_panel::SauronPanel, rviz_common::Panel)
