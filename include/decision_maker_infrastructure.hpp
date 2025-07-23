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
 *    Marko Mizdrak
 ********************************************************************************/

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "adore_dynamics_adapters.hpp"
#include "adore_dynamics_conversions.hpp"
#include "adore_map/map.hpp"
#include "adore_map/map_loader.hpp"
#include "adore_map/route.hpp"
#include "adore_map/traffic_light.hpp"
#include "adore_map_conversions.hpp"
#include "adore_math/angles.h"
#include "adore_math/distance.h"
#include "adore_math/polygon.h"
#include "adore_ros2_msgs/msg/infrastructure_info.hpp"
#include "adore_ros2_msgs/msg/map.hpp"
#include "adore_ros2_msgs/msg/route.hpp"
#include "adore_ros2_msgs/msg/traffic_participant_set.hpp"
#include "adore_ros2_msgs/msg/visualizable_object.hpp"

#include "planning/multi_agent_PID.hpp"
#include "planning/multi_agent_planner.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


using namespace std::chrono_literals;

namespace adore
{

class DecisionMakerInfrastructure : public rclcpp::Node
{
private:

  rclcpp::TimerBase::SharedPtr                                           main_timer;
  rclcpp::Publisher<ParticipantSetAdapter>::SharedPtr                    publisher_planned_traffic;
  rclcpp::Publisher<adore_ros2_msgs::msg::VisualizableObject>::SharedPtr publisher_infrastructure_position;
  rclcpp::Publisher<adore_ros2_msgs::msg::InfrastructureInfo>::SharedPtr publisher_infrastructure_info;

  using StateSubscriber = rclcpp::Subscription<ParticipantAdapter>::SharedPtr;
  std::unordered_map<std::string, StateSubscriber> traffic_participant_subscribers;

  std::shared_ptr<map::Map>              road_map = nullptr;
  adore::dynamics::TrafficParticipantSet latest_traffic_participant_set;

public:

  double              dt             = 0.1;
  double              local_map_size = 200;
  adore::math::Pose2d infrastructure_pose;

  void run();
  void update_state();
  void create_subscribers();
  void create_publishers();
  void load_parameters();
  void print_init_info();
  void print_debug_info();
  void plan_traffic();
  void update_dynamic_subscriptions();


  /******************************* PUBLISHER RELATED FUNCTIONS ************************************************************/
  void publish_infrastructure_position();

  /******************************* SUBSCRIBER RELATED FUNCTIONS************************************************************/
  void traffic_participant_callback( const dynamics::TrafficParticipant& msg, const std::string& vehicle_namespace );

  explicit DecisionMakerInfrastructure( const rclcpp::NodeOptions& options );
};

} // namespace adore
