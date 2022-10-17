#include "kortex_joystick_demo/protection_zones_node.hpp"

bool shutdown_requested = false;

ProtectionZonesNode::ProtectionZonesNode(ros::NodeHandle * nh, ros::NodeHandle * private_nh)
{
  signal(SIGINT, mySigintHandler);
  // signal(SIGTERM, mySigintHandler);
  node_name_ = ros::this_node::getName();
  node_name_ = node_name_.substr(1, node_name_.length());

  create_protection_zone_client_ =
    nh->serviceClient<kortex_driver::CreateProtectionZone>("base/create_protection_zone");
  delete_protection_zone_client_ =
    nh->serviceClient<kortex_driver::DeleteProtectionZone>("base/delete_protection_zone");
  read_all_protection_zones_client_ =
    nh->serviceClient<kortex_driver::ReadAllProtectionZones>("base/read_all_protection_zones");

  ROS_INFO("[%s] Node started", node_name_.c_str());

  manage_protection_zones();
}

void ProtectionZonesNode::manage_protection_zones()
{
  // create protection zones
  create_protection_zone(
    "panther_protection_zone", 2, "box", {0.95, 1.0, 0.44}, {-0.135, 0.1725, -0.18});
  create_protection_zone(
    "ouster_protection_zone", 3, "cylinder", {0.12, 0.12}, {-0.32, 0.1725, 0.06});

  while (!shutdown_requested) {
    ros::Duration(1.0).sleep();
  }

  // delete protection zones
  delete_protection_zones();

  ros::shutdown();
}

void ProtectionZonesNode::create_protection_zone(
  std::string name, int id, std::string shape, std::vector<float> dimensions,
  std::vector<float> position)
{
  get_protection_zones();
  auto protection_zones = read_all_protection_zones_srv_.response;
  for (auto & zone : protection_zones.output.protection_zones) {
    if (zone.name == name) return;
  }

  ROS_INFO(
    "[%s] Creating protection zone '%s' of type '%s'", node_name_.c_str(), name.c_str(),
    shape.c_str());

  int shape_type;
  kortex_driver::CreateProtectionZoneRequest protection_zone;

  if (shape == "cylinder")
    shape_type = 1;
  else if (shape == "sphere")
    shape_type = 2;
  else if (shape == "box")
    shape_type = 3;
  else {
    ROS_ERROR(
      "[%s] Invalid shape type. Valid options are: 'cylinder', 'sphere', 'box'",
      node_name_.c_str());
    return;
  }

  protection_zone.input.name = name;
  protection_zone.input.handle.identifier = id;
  protection_zone.input.handle.permission = 7;
  protection_zone.input.is_enabled = true;
  // Shape and position
  protection_zone.input.shape.shape_type = shape_type;
  protection_zone.input.shape.dimensions = dimensions;
  protection_zone.input.shape.origin.x = position.at(0);
  protection_zone.input.shape.origin.y = position.at(1);
  protection_zone.input.shape.origin.z = position.at(2);
  protection_zone.input.shape.orientation.row1.column1 = 1.0;
  protection_zone.input.shape.orientation.row1.column2 = 0.0;
  protection_zone.input.shape.orientation.row1.column3 = 0.0;
  protection_zone.input.shape.orientation.row2.column1 = 0.0;
  protection_zone.input.shape.orientation.row2.column2 = 1.0;
  protection_zone.input.shape.orientation.row2.column3 = 0.0;
  protection_zone.input.shape.orientation.row3.column1 = 0.0;
  protection_zone.input.shape.orientation.row3.column2 = 0.0;
  protection_zone.input.shape.orientation.row3.column3 = 1.0;
  protection_zone.input.shape.envelope_thickness = 0.0;

  create_protection_zone_srv_.request = protection_zone;

  create_protection_zone_client_.waitForExistence(ros::Duration(5));
  if (create_protection_zone_client_.call(create_protection_zone_srv_)) {
    zone_names_.push_back(name);
    ROS_INFO("[%s] Protecrion zone '%s' created", node_name_.c_str(), name.c_str());
  } else {
    ROS_ERROR("Failed to call service base/create_protection_zone");
  }
}

void ProtectionZonesNode::delete_protection_zones()
{
  get_protection_zones();
  auto protection_zones = read_all_protection_zones_srv_.response;

  for (auto & zone : protection_zones.output.protection_zones) {
    for (auto & zone_name : zone_names_) {
      if (zone.name == zone_name) delete_protection_zone(zone.handle.identifier);
    }
  }
}

void ProtectionZonesNode::get_protection_zones()
{
  read_all_protection_zones_client_.waitForExistence(ros::Duration(5));
  if (read_all_protection_zones_client_.call(read_all_protection_zones_srv_)) {
    ROS_INFO("[%s] Protection zones read", node_name_.c_str());
  } else {
    ROS_ERROR("Failed to call service base/read_all_protection_zones");
  }
}

void ProtectionZonesNode::delete_protection_zone(int zone_id)
{
  kortex_driver::DeleteProtectionZone::Request protection_zone;
  protection_zone.input.identifier = zone_id;

  delete_protection_zone_srv_.request = protection_zone;

  delete_protection_zone_client_.waitForExistence(ros::Duration(5));
  if (delete_protection_zone_client_.call(delete_protection_zone_srv_)) {
    ROS_INFO(
      "[%s] Protecrion zone with id '%s' deleted", node_name_.c_str(),
      std::to_string(zone_id).c_str());
  } else {
    ROS_ERROR("Failed to call service base/delete_protection_zone");
  }
}

void ProtectionZonesNode::mySigintHandler(int sig) { shutdown_requested = true; }