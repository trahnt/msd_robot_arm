#pragma once

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
// RVIZ2
#include <rviz_common/panel.hpp>
// Qt
#include <QtWidgets>
// STL
#include <memory>
/** 
 *  Include header generated from ui file
 *  Note that you will need to use add_library function first
 *  in order to generate the header file from ui.
 */
#include <ui_push_button.h>

namespace custom_panel
{
  class SauronPanel : public rviz_common::Panel
  {
    Q_OBJECT
  public:
    explicit SauronPanel(QWidget *parent = nullptr);
    ~SauronPanel();

    /// Load and save configuration data
    virtual void load(const rviz_common::Config &config) override;
    virtual void save(rviz_common::Config config) const override;


  // for these, the name needs to match the UI component, see the .ui file to
  // confirm
  private Q_SLOTS:
    // void on_<BUTTON_NAME>_clicked();
    void on_homeButton_clicked();

  private:
    std::unique_ptr<Ui::gui> ui_;
    rclcpp::Node::SharedPtr node_;

  protected:
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr homerPub_;
    std_msgs::msg::Bool msg_;
  };
} // custom_panel
