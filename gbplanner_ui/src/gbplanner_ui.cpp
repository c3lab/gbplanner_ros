#include "gbplanner_ui.h"

#include <rviz_common/display_context.hpp>

namespace gbplanner_ui {
namespace {

// A panel slot runs on RViz's executor thread, so waiting on a service future
// here would deadlock the GUI: the executor cannot deliver the response while it
// is blocked inside this callback. Fire and forget instead, and report failure
// from the completion callback. The empty lambda body is still required - it is
// what lets rclcpp retire the pending request.
template <typename ClientT>
void callAsync(const ClientT& client, rclcpp::Logger logger,
               typename ClientT::element_type::Request::SharedPtr request) {
  if (!client->service_is_ready()) {
    RCLCPP_ERROR(logger, "[GBPLANNER-UI] Service unavailable: %s",
                 client->get_service_name());
    return;
  }
  client->async_send_request(
      request, [](typename ClientT::element_type::SharedFuture) {});
}

}  // namespace

gbplanner_panel::gbplanner_panel(QWidget* parent) : rviz_common::Panel(parent) {
  QVBoxLayout* v_box_layout = new QVBoxLayout;

  button_start_planner = new QPushButton;
  button_start_planner_single = new QPushButton;
  button_stop_planner = new QPushButton;
  button_homing = new QPushButton;
  button_init_motion = new QPushButton;
  button_plan_to_waypoint = new QPushButton;
  button_global_planner = new QPushButton;
  button_change_operation_mode = new QPushButton;

  button_start_planner->setText("Start Planner");
  button_start_planner_single->setText("Start Single Planner");
  button_stop_planner->setText("Stop Planner");
  button_homing->setText("Go Home");
  button_init_motion->setText("Initialization");
  button_plan_to_waypoint->setText("Plan to Waypoint");
  button_global_planner->setText("Run Global");
  button_change_operation_mode->setText("Operation Mode (EXP)");

  v_box_layout->addWidget(button_start_planner);
  v_box_layout->addWidget(button_start_planner_single);
  v_box_layout->addWidget(button_stop_planner);
  v_box_layout->addWidget(button_homing);
  v_box_layout->addWidget(button_init_motion);
  v_box_layout->addWidget(button_plan_to_waypoint);
  v_box_layout->addWidget(button_change_operation_mode);

  QVBoxLayout* global_vbox_layout = new QVBoxLayout;
  QHBoxLayout* global_hbox_layout = new QHBoxLayout;

  QLabel* text_label_ptr = new QLabel("Frontier ID:");

  global_id_line_edit = new QLineEdit();

  global_hbox_layout->addWidget(text_label_ptr);
  global_hbox_layout->addWidget(global_id_line_edit);
  global_hbox_layout->addWidget(button_global_planner);
  global_vbox_layout->addLayout(global_hbox_layout);
  v_box_layout->addLayout(global_vbox_layout);

  setLayout(v_box_layout);

  connect(button_start_planner, SIGNAL(clicked()), this,
          SLOT(on_start_planner_click()));
  connect(button_start_planner_single, SIGNAL(clicked()), this,
          SLOT(on_start_planner_single_click()));
  connect(button_stop_planner, SIGNAL(clicked()), this,
          SLOT(on_stop_planner_click()));
  connect(button_homing, SIGNAL(clicked()), this, SLOT(on_homing_click()));
  connect(button_init_motion, SIGNAL(clicked()), this,
          SLOT(on_init_motion_click()));
  connect(button_plan_to_waypoint, SIGNAL(clicked()), this,
          SLOT(on_plan_to_waypoint_click()));
  connect(button_global_planner, SIGNAL(clicked()), this,
          SLOT(on_global_planner_click()));
  connect(button_change_operation_mode, SIGNAL(clicked()), this,
          SLOT(on_change_operation_mode_click()));
}

void gbplanner_panel::onInitialize() {
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  planner_client_start_planner = node_->create_client<std_srvs::srv::Trigger>(
      "/planner_control_interface/std_srvs/automatic_planning");
  planner_client_start_planner_single =
      node_->create_client<std_srvs::srv::Trigger>(
          "/planner_control_interface/std_srvs/single_planning");
  planner_client_stop_planner = node_->create_client<std_srvs::srv::Trigger>(
      "/planner_control_interface/std_srvs/stop");
  planner_client_homing = node_->create_client<std_srvs::srv::Trigger>(
      "/planner_control_interface/std_srvs/homing_trigger");
  planner_client_init_motion =
      node_->create_client<planner_msgs::srv::PciInitialization>(
          "pci_initialization_trigger");
  planner_client_plan_to_waypoint =
      node_->create_client<std_srvs::srv::Trigger>(
          "/planner_control_interface/std_srvs/go_to_waypoint");
  planner_client_global_planner =
      node_->create_client<planner_msgs::srv::PciGlobal>("pci_global");
  change_operation_mode_client = node_->create_client<std_srvs::srv::SetBool>(
      "gbplanner/switch_operation_mode");
}

void gbplanner_panel::on_start_planner_click() {
  callAsync(planner_client_start_planner, node_->get_logger(),
            std::make_shared<std_srvs::srv::Trigger::Request>());
}

void gbplanner_panel::on_start_planner_single_click() {
  callAsync(planner_client_start_planner_single, node_->get_logger(),
            std::make_shared<std_srvs::srv::Trigger::Request>());
}

void gbplanner_panel::on_stop_planner_click() {
  callAsync(planner_client_stop_planner, node_->get_logger(),
            std::make_shared<std_srvs::srv::Trigger::Request>());
}

void gbplanner_panel::on_homing_click() {
  callAsync(planner_client_homing, node_->get_logger(),
            std::make_shared<std_srvs::srv::Trigger::Request>());
}

void gbplanner_panel::on_init_motion_click() {
  callAsync(planner_client_init_motion, node_->get_logger(),
            std::make_shared<planner_msgs::srv::PciInitialization::Request>());
}

void gbplanner_panel::on_plan_to_waypoint_click() {
  callAsync(planner_client_plan_to_waypoint, node_->get_logger(),
            std::make_shared<std_srvs::srv::Trigger::Request>());
}

void gbplanner_panel::on_global_planner_click() {
  // retrieve ID as a string
  std::string in_string = global_id_line_edit->text().toStdString();
  // global_id_line_edit->clear();
  int id = -1;
  if (in_string.empty())
    id = 0;
  else {
    // try to convert to an integer
    try {
      id = std::stoi(in_string);
    } catch (const std::out_of_range& exc) {
      RCLCPP_ERROR(node_->get_logger(), "[GBPLANNER UI] - Invalid ID: %s",
                   in_string.c_str());
      return;
    } catch (const std::invalid_argument& exc) {
      RCLCPP_ERROR(node_->get_logger(), "[GBPLANNER UI] - Invalid ID: %s",
                   in_string.c_str());
      return;
    }
  }
  // check bounds on integer
  if (id < 0) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[GBPLANNER UI] - In valid ID, must be non-negative");
    return;
  }
  // we got an ID!!!!!!!!!
  RCLCPP_INFO(node_->get_logger(), "Global Planner found ID : %i", id);

  auto plan_req = std::make_shared<planner_msgs::srv::PciGlobal::Request>();
  plan_req->id = id;
  callAsync(planner_client_global_planner, node_->get_logger(), plan_req);
}

void gbplanner_panel::on_change_operation_mode_click() {
  waypoint_nav_mode = !waypoint_nav_mode;
  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = waypoint_nav_mode;
  callAsync(change_operation_mode_client, node_->get_logger(), req);

  if (waypoint_nav_mode) {
    button_change_operation_mode->setText("Operation mode (WP)");
  } else {
    button_change_operation_mode->setText("Operation mode (EXP)");
  }
}

void gbplanner_panel::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);
}
void gbplanner_panel::load(const rviz_common::Config& config) {
  rviz_common::Panel::load(config);
}

}  // namespace gbplanner_ui

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(gbplanner_ui::gbplanner_panel, rviz_common::Panel)
