#ifndef RVIZ_PANEL_HPP
#define RVIZ_PANEL_HPP

#include <rviz_common/panel.hpp>
#include <QLabel>
#include <QPushButton>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <std_msgs/msg/string.hpp>
#include "std_msgs/msg/bool.hpp"
#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>
namespace driverstation{

    class RvizPanel : public rviz_common::Panel{

        Q_OBJECT
        public:
            explicit RvizPanel(QWidget* parent = 0);

            ~RvizPanel() override;

            void onInitialize() override;

            bool is_enabled();
            void toggle_enabled(bool enable);

        protected:
            std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_ptr;

            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher;
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription;

            QVBoxLayout* layout;

            QPushButton* button_disable;
            QPushButton* button_enable;

            void resizeEvent(QResizeEvent* event) override;

        private:
            bool enabled = false;

        private Q_SLOTS:
            // slots you wanna put
          

    }; // class RvizPanel

} // namespace driverstation

#endif // RVIZ_PANEL_HPP