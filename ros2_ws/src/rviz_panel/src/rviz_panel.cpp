#include "rviz_panel/rviz_panel.hpp"

namespace driverstation{
    RvizPanel::RvizPanel(QWidget* parent) : Panel(parent){

        layout = new QVBoxLayout(this);

        button_disable = new QPushButton("DISABLE");
        button_enable = new QPushButton("ENABLE");

        layout->addWidget(button_disable);
        layout->addWidget(button_enable);

        QObject::connect(button_disable, &QPushButton::pressed, this, 
            [this](){
                toggle_enabled(false);

                std_msgs::msg::Bool msg;
                    msg.data = false;

                publisher->publish(msg);
            }
        );
        QObject::connect(button_enable, &QPushButton::pressed, this,
            [this](){
                toggle_enabled(true);

                std_msgs::msg::Bool msg;
                    msg.data = true;

                publisher->publish(msg);
            }
        );
    }

    RvizPanel::~RvizPanel() = default;

    bool RvizPanel::is_enabled(){
        return enabled; 
    }

    void RvizPanel::toggle_enabled(bool enable){
        enabled = enable;

        // if you want to enable
        if(enabled){
            button_enable->setStyleSheet("background-color: green");
            button_disable->setStyleSheet("background-color: gray");
        }

        // if you want to disable
        else{
            button_disable->setStyleSheet("background-color: red");
            button_enable->setStyleSheet("background-color: gray");
        }
    }

    void RvizPanel::resizeEvent(QResizeEvent* event){
        QWidget::resizeEvent(event);

        if(layout == nullptr)
            return;

        int height = layout->geometry().height();
        int width = layout->geometry().width();

        button_disable->setFixedHeight(0.8 * (height / 2));
        button_enable->setFixedHeight(0.8 * (height / 2));

        // if(node_ptr == nullptr)
        //     return;

        // RCLCPP_INFO(node_ptr->get_raw_node()->get_logger(), std::to_string(width).c_str());
    }

    void RvizPanel::onInitialize(){
        node_ptr = getDisplayContext()->getRosNodeAbstraction().lock();

        rclcpp::Node::SharedPtr node = node_ptr->get_raw_node();

        publisher = node->create_publisher<std_msgs::msg::Bool>("/enabled", rclcpp::SystemDefaultsQoS());
        subscription = node->create_subscription<std_msgs::msg::Bool>(
            "/enabled", rclcpp::SystemDefaultsQoS(),
            [this](const std_msgs::msg::Bool& msg){
                bool enable = msg.data;

                toggle_enabled(enable);
            }
        );

        toggle_enabled(false);
    }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(driverstation::RvizPanel, rviz_common::Panel)