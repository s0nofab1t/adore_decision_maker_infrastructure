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

#include "decision_maker_infrastructure.hpp"

#include <adore_dynamics_conversions.hpp>
#include <adore_math/point.h>

namespace adore
{

DecisionMakerInfrastructure::DecisionMakerInfrastructure( const rclcpp::NodeOptions& options ) :
  Node( "decision_maker_infrastructure", options )
{
  load_parameters();
  create_subscribers();
  create_publishers();
  print_init_info();
}

void
DecisionMakerInfrastructure::run()
{
  auto start_time = std::chrono::high_resolution_clock::now(); // Start timer
  plan_traffic();
  auto   end_time        = std::chrono::high_resolution_clock::now(); // End timer
  double elapsed_time_ms = std::chrono::duration<double, std::milli>( end_time - start_time ).count();

  RCLCPP_INFO( this->get_logger(), "Planning took %.3f ms", elapsed_time_ms );

  print_debug_info();
  publish_infrastructure_position();
}

void
DecisionMakerInfrastructure::plan_traffic()
{
  if( !road_map )
    return;

  planner::MultiAgentPlanner planner;
  latest_traffic_participant_set = planner.plan_all_participants( latest_traffic_participant_set, road_map );
  auto participants_no_routes    = latest_traffic_participant_set;
  for( auto& [id, participant] : participants_no_routes.participants )
  {
    participant.route.reset();
  }
  publisher_planned_traffic->publish( participants_no_routes );
}

void
DecisionMakerInfrastructure::create_subscribers()
{
  update_dynamic_subscriptions();
  main_timer = create_wall_timer( 100ms, std::bind( &DecisionMakerInfrastructure::run, this ) );
}

void
DecisionMakerInfrastructure::create_publishers()
{
  publisher_planned_traffic         = create_publisher<ParticipantSetAdapter>( "/planned_traffic", 1 );
  publisher_infrastructure_position = create_publisher<adore_ros2_msgs::msg::VisualizableObject>( "infrastructure_position", 1 );
  publisher_infrastructure_info     = create_publisher<adore_ros2_msgs::msg::InfrastructureInfo>( "infrastructure_info", 1 );
}

void
DecisionMakerInfrastructure::load_parameters()
{
  // Debug and simulation parameters
  dt = declare_parameter<double>( "dt", 0.1 );

  std::string map_file_location = declare_parameter<std::string>( "map file", "" );
  road_map                      = std::make_shared<map::Map>( map::MapLoader::load_from_file( map_file_location ) );

  infrastructure_pose.x   = declare_parameter<double>( "infrastructure_position_x", 0.0 );
  infrastructure_pose.y   = declare_parameter<double>( "infrastructure_position_y", 0.0 );
  infrastructure_pose.yaw = declare_parameter<double>( "infrastructure_yaw", 0.0 );

  // Validity area polygon
  std::vector<double> validity_area_points;
  validity_area_points = declare_parameter<std::vector<double>>( "validity_polygon", validity_area_points );

  // Convert parameter into Polygon2d if valid
  if( validity_area_points.size() >= 6 ) // minimum 3 points (x,y)
  {
    math::Polygon2d validity_area;
    validity_area.points.reserve( validity_area_points.size() / 2 );

    for( size_t i = 0; i < validity_area_points.size(); i += 2 )
    {
      const double x = validity_area_points[i];
      const double y = validity_area_points[i + 1];
      validity_area.points.push_back( { x, y } );
    }

    latest_traffic_participant_set.validity_area = validity_area;
  }
}

void
DecisionMakerInfrastructure::print_init_info()
{
  RCLCPP_INFO( get_logger(), "DecisionMakerInfrastructure node initialized." );
}

void
DecisionMakerInfrastructure::print_debug_info()
{
  const double current_time_seconds = now().seconds();

  RCLCPP_DEBUG( get_logger(), "------- Decision Maker Infrastructure Debug Information -------" );
  RCLCPP_DEBUG( get_logger(), "Current Time: %.3f seconds", current_time_seconds );

  if( road_map )
    RCLCPP_DEBUG( get_logger(), "Local map data available." );
  else
    RCLCPP_DEBUG( get_logger(), "No local map data." );

  RCLCPP_DEBUG( get_logger(), "Traffic participants: %zu", latest_traffic_participant_set.participants.size() );
  RCLCPP_DEBUG( get_logger(), "------- ============================== -------" );
}

void
DecisionMakerInfrastructure::publish_infrastructure_position()
{
  adore_ros2_msgs::msg::VisualizableObject obj;
  obj.x               = infrastructure_pose.x;
  obj.y               = infrastructure_pose.y;
  obj.yaw             = infrastructure_pose.yaw;
  obj.z               = 0.0;
  obj.model           = "low_poly_trailer_model.dae";
  obj.header.frame_id = "world";

  publisher_infrastructure_position->publish( obj );

  adore_ros2_msgs::msg::InfrastructureInfo infrastructure_info_msg;
  infrastructure_info_msg.position_x = infrastructure_pose.x;
  infrastructure_info_msg.position_y = infrastructure_pose.y;
  infrastructure_info_msg.yaw        = infrastructure_pose.yaw;

  adore_ros2_msgs::msg::Polygon2d validity_area_msg;
  if( latest_traffic_participant_set.validity_area.has_value() )
  {
    for( const auto& point : latest_traffic_participant_set.validity_area->points )
    {
      adore_ros2_msgs::msg::Point2d p;
      p.x = point.x;
      p.y = point.y;
      validity_area_msg.points.push_back( p );
    }

    infrastructure_info_msg.validity_area = validity_area_msg;
  }
  publisher_infrastructure_info->publish( infrastructure_info_msg );
}

void
DecisionMakerInfrastructure::traffic_participant_callback( const dynamics::TrafficParticipant& msg, const std::string& vehicle_namespace )
{
  latest_traffic_participant_set.update_traffic_participants( msg );
}

void
DecisionMakerInfrastructure::update_dynamic_subscriptions()
{
  auto       topic_names_and_types = get_topic_names_and_types();
  std::regex valid_topic_regex( R"(^/([^/]+)/traffic_participant$)" );
  std::regex valid_type_regex( R"(^adore_ros2_msgs/msg/TrafficParticipant$)" );

  for( const auto& topic : topic_names_and_types )
  {
    const std::string&              topic_name = topic.first;
    const std::vector<std::string>& types      = topic.second;

    std::smatch match;
    if( std::regex_match( topic_name, match, valid_topic_regex )
        && std::any_of( types.begin(), types.end(),
                        [&]( const std::string& type ) { return std::regex_match( type, valid_type_regex ); } ) )
    {
      std::string vehicle_namespace = match[1].str();

      // Skip subscribing to own namespace
      if( vehicle_namespace == std::string( get_namespace() ).substr( 1 ) )
      {
        continue;
      }

      // Check if already subscribed
      if( traffic_participant_subscribers.count( vehicle_namespace ) > 0 )
      {
        continue;
      }

      // Create a new subscription
      auto subscription = create_subscription<ParticipantAdapter>( topic_name, 10,
                                                                   [this, vehicle_namespace]( const dynamics::TrafficParticipant& msg ) {
        traffic_participant_callback( msg, vehicle_namespace );
      } );

      traffic_participant_subscribers[vehicle_namespace] = subscription;

      RCLCPP_INFO( get_logger(), "Subscribed to new vehicle namespace: %s", vehicle_namespace.c_str() );
    }
  }
}

}; // namespace adore

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE( adore::DecisionMakerInfrastructure )
