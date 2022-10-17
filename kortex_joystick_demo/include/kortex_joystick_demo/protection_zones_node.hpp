#ifndef KORTEX_JOYSTICK_DEMO_PANTHER_SAFETY_ZONES_NODE_HPP_
#define KORTEX_JOYSTICK_DEMO_PANTHER_SAFETY_ZONES_NODE_HPP_

#include <kortex_driver/CreateProtectionZone.h>
#include <kortex_driver/DeleteProtectionZone.h>
#include <kortex_driver/ReadProtectionZone.h>
#include <kortex_driver/ReadAllProtectionZones.h>
#include <ros/ros.h>
#include <signal.h>
#include <std_srvs/SetBool.h>

class ProtectionZonesNode
{
public:
  ProtectionZonesNode(ros::NodeHandle * nh, ros::NodeHandle * private_nh);

private:
  ros::ServiceClient create_protection_zone_client_;
  ros::ServiceClient delete_protection_zone_client_;
  ros::ServiceClient read_all_protection_zones_client_;
  kortex_driver::CreateProtectionZone create_protection_zone_srv_;
  kortex_driver::DeleteProtectionZone delete_protection_zone_srv_;
  kortex_driver::ReadAllProtectionZones read_all_protection_zones_srv_;

  std::string node_name_;
  std::vector<std::string> zone_names_;

  void manage_protection_zones();
  void create_protection_zone(
    std::string name, int zone_id, std::string shape, std::vector<float> dimensions,
    std::vector<float> position);
  void delete_protection_zones();
  void get_protection_zones();
  void delete_protection_zone(int zone_id);

  static void mySigintHandler(int sig);
};

#endif  // KORTEX_JOYSTICK_DEMO_KORTEX_JOYSTICK_NODE_HPP_