/********************************************************************************
 * Copyright (C) 2017-2020 German Aerospace Center (DLR).
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contributors:
 *    Giovanni Lucente
 ********************************************************************************/

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "adore_dynamics_conversions.hpp"
#include "adore_map/map.hpp"
#include "adore_map/map_loader.hpp"
#include "adore_map/route.hpp"
#include "adore_map/traffic_light.hpp"
#include "adore_map_conversions.hpp"
#include "adore_math/angles.h"
#include "adore_math/distance.h"
#include "adore_math/polygon.h"
#include "adore_ros2_msgs/msg/map.hpp"
#include "adore_ros2_msgs/msg/route.hpp"
#include "adore_ros2_msgs/msg/traffic_participant_set.hpp"
#include "adore_ros2_msgs/msg/traffic_signals.hpp"
#include "adore_ros2_msgs/msg/visualizable_object.hpp"
#include "adore_ros2_msgs/msg/infrastructure_info.hpp"

#include "planning/multi_agent_PID.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace adore
{

class DecisionMakerInfrastructure : public rclcpp::Node
{
private:

  /******************************* PUBLISHERS RELATED MEMBERS ************************************************************/
  rclcpp::TimerBase::SharedPtr                                              main_timer;
  rclcpp::Publisher<adore_ros2_msgs::msg::InfrastructureInfo>::SharedPtr publisher_infrastructure_info;
  rclcpp::Publisher<adore_ros2_msgs::msg::TrafficParticipantSet>::SharedPtr publisher_planned_traffic;
  rclcpp::Publisher<adore_ros2_msgs::msg::Map>::SharedPtr                   publisher_local_map;
  rclcpp::Publisher<adore_ros2_msgs::msg::VisualizableObject>::SharedPtr    publisher_infrastructure_position;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    publisher_overview;

  /******************************* SUBSCRIBERS RELATED MEMBERS ************************************************************/
  rclcpp::Subscription<adore_ros2_msgs::msg::TrafficParticipantSet>::SharedPtr subscriber_traffic_participant_set;
  rclcpp::Subscription<adore_ros2_msgs::msg::TrafficSignals>::SharedPtr        subscriber_traffic_signals;

  /******************************* OTHER MEMBERS *************************************************************************/
  std::optional<adore::map::Map>         road_map = std::nullopt;
  std::optional<adore::map::Map>         local_map = std::nullopt;
  adore::dynamics::TrafficParticipantSet latest_traffic_participant_set;
  std::string overview;

public:

  bool                                  debug_mode_active = true;
  double                                dt                = 0.1;
  double                                local_map_size    = 200;
  std::string                           traffic_participants_topic;
  adore::math::Pose2d                   infrastructure_pose;
  adore::dynamics::VehicleCommandLimits command_limits = { 0.7, -2.0, 2.0 };
  std::map<std::string, double>         multi_agent_PID_settings;
  adore::planner::MultiAgentPID         multi_agent_PID_planner;
  adore::planner::MultiAgentPID         multi_agent_PID_planner_MRM;

  // intermediate solution for traffic lights
  std::vector<adore::math::Point2d> stopping_points;

  std::string map_file_location;

  void run();
  void update_state();
  void create_subscribers();
  void create_publishers();
  void load_parameters();
  void print_init_info();
  void debug_info(bool print);
  void compute_routes_for_traffic_participant_set( adore::dynamics::TrafficParticipantSet& traffic_participant_set,
                                                   const adore::map::Map&                  road_map );
  void all_vehicles_follow_routes();

  /******************************* PUBLISHER RELATED FUNCTIONS ************************************************************/
  void publish_local_map();
  void publish_infrastructure_position();
  void publish_infrastructure_info();

  /******************************* SUBSCRIBER RELATED FUNCTIONS************************************************************/
  void traffic_participants_callback( const adore_ros2_msgs::msg::TrafficParticipantSet& msg );
  void traffic_signals_callback( const adore_ros2_msgs::msg::TrafficSignals& msg );

  explicit DecisionMakerInfrastructure(const rclcpp::NodeOptions & options);
};

} // namespace adore
