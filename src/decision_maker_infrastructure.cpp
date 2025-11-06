/********************************************************************************
 * Copyright (c) 2025 Contributors to the Eclipse Foundation
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

#include "decision_maker_infrastructure.hpp"

#include <adore_dynamics_conversions.hpp>
#include <adore_math/point.h>
#include <adore_math/polygon.h>
#include "std_msgs/msg/string.hpp"

#include <planning/planning_helpers.hpp>

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

  if( debug )
  {
    print_debug_info();
  }

  publish_infrastructure_position();
}

void
DecisionMakerInfrastructure::plan_traffic()
{
  if( !road_map )
    return;

  latest_traffic_participant_set.remove_old_participants( max_participant_age, now().seconds() );

  if( latest_traffic_participant_set.participants.empty() )
    return;


  auto make_valid_route = [&]( const dynamics::VehicleStateDynamic& start_state,
                               const std::optional<math::Point2d>&  goal ) -> std::optional<map::Route> {
    if( goal )
    {
      map::Route r_goal( start_state, *goal, road_map );
      if( !r_goal.center_lane.empty() )
        return r_goal;
    }
    else
    {
      map::Route r_def = map::get_default_route( start_state, max_route_length, road_map );
      if( !r_def.center_lane.empty() )
        return r_def;
    }

    return std::nullopt;
  };

  auto needs_replan = [&]( const std::optional<map::Route>& route, const dynamics::VehicleStateDynamic& start_state ) {
    if( !route )
      return true;
    const double s = route->get_s( start_state );
    return route->center_lane.empty() || ( route->get_length() > 0.0 && s >= route->get_length() - route_replan_dist );
  };

  // ---------- Route calculation timing ----------
  const auto t_route_begin = std::chrono::steady_clock::now();

  std::size_t n_considered = 0;
  std::size_t n_replanned  = 0;
  std::size_t n_goal_used  = 0;
  std::size_t n_def_used   = 0;

  for( auto& [id, p] : latest_traffic_participant_set.participants )
  {
    if( p.classification == dynamics::PEDESTRIAN )
      continue;
    ++n_considered;

    if( needs_replan( p.route, p.state ) )
    {
      const bool had_goal = static_cast<bool>( p.goal_point );
      auto       before   = p.route.has_value();

      auto r = make_valid_route( p.state, p.goal_point );
      if( r )
      {
        // count whether we used a goal-route or default-route
        if( had_goal )
        {
          ++n_goal_used;
        }
        else
        {
          ++n_def_used;
        }
        p.route = std::move( *r );
      }
      else
      {
        p.route.reset();
      }

      ++n_replanned;
      RCLCPP_DEBUG( get_logger(), "Replanned route for participant %ld (had=%d, goal=%d).", static_cast<long>( id ), before ? 1 : 0,
                    had_goal ? 1 : 0 );
    }
  }

  const auto   t_route_end = std::chrono::steady_clock::now();
  const double route_ms    = std::chrono::duration<double, std::milli>( t_route_end - t_route_begin ).count();

  // RCLCPP_INFO( get_logger(), "Route calc: %.1f ms | considered=%zu replanned=%zu goal_used~=%zu default_used~=%zu participants=%zu",
  //              route_ms, n_considered, n_replanned, n_goal_used, n_def_used, latest_traffic_participant_set.participants.size() );

  // ---------- Planning timing ----------
  const auto t_plan_begin = std::chrono::steady_clock::now();

  // planner::MultiAgentPID pid_planner;

  // pid_planner.plan_trajectories( latest_traffic_participant_set );

  for( auto& [id, participant] : latest_traffic_participant_set.participants )
  {
    if( !participant.route )
    {
      RCLCPP_WARN( get_logger(), "Participant %d has no route, skipping trajectory planning.", participant.id );
      continue;
    }
    double participant_s = participant.route->get_s( participant.state );

    std::vector<map::MapPoint> points;
    for( const auto& [s, point] : participant.route->center_lane )
    {
      if( s > participant_s + 0.5 ) // start a bit ahead of the vehicle
      {
        points.push_back( point );
      }
    }
    auto motion_model = [params = participant.physical_parameters]( const dynamics::VehicleStateDynamic& state,
                                                                    const dynamics::VehicleCommand& cmd ) -> dynamics::VehicleStateDynamic {
      return dynamics::kinematic_bicycle_model( state, params, cmd );
    };
    auto model         = dynamics::PhysicalVehicleModel();
    model.params       = participant.physical_parameters;
    model.motion_model = motion_model;

    auto participants_copy = latest_traffic_participant_set;
    participants_copy.participants.erase( id ); // remove self from obstacles

    participant.trajectory = planner::waypoints_to_trajectory( participant.state, points, participants_copy, model, 10.0, dt );
  }

  planner::MultiAgentPlanner planner;

  planner.set_parameters( {
    {                       "dt",   dt },
    {            "horizon_steps",   30 },
    {                "max_speed", 10.0 },
    { "max_lateral_acceleration",  2.0 }
  } );

  auto planned = planner.plan_all_participants( latest_traffic_participant_set, road_map );

  const auto   t_plan_end = std::chrono::steady_clock::now();
  const double plan_ms    = std::chrono::duration<double, std::milli>( t_plan_end - t_plan_begin ).count();

  // RCLCPP_INFO( get_logger(), "Planning: %.1f ms for %zu participants", plan_ms, planned.participants.size() );

  // ---------- Submap attach ----------
  auto sub = road_map->get_submap( infrastructure_pose, 200, 200 );
  for( auto& [id, p] : planned.participants )
    if( p.route )
      p.route->map = std::make_shared<map::Map>( sub );

  publisher_planned_traffic->publish( planned );
  // publisher_planned_traffic->publish( latest_traffic_participant_set );
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
  publisher_planned_traffic         = create_publisher<ParticipantSetAdapter>( planned_traffic_out_topic, 1 );
  publisher_infrastructure_position = create_publisher<adore_ros2_msgs::msg::VisualizableObject>( "infrastructure_position", 1 );
  publisher_infrastructure_info     = create_publisher<adore_ros2_msgs::msg::InfrastructureInfo>( "infrastructure_info", 1 );
}

void
DecisionMakerInfrastructure::load_parameters()
{
  planned_traffic_out_topic    = declare_parameter<std::string>( "planned_traffic_out_topic", "/planned_traffic" );
  traffic_participant_in_topic = declare_parameter<std::string>( "traffic_participant_in_topic", "/traffic_participant" );
  // Debug and simulation parameters
  dt = declare_parameter<double>( "dt", 0.1 );

  std::string map_file_location = declare_parameter<std::string>( "map file", "" );
  road_map                      = std::make_shared<map::Map>( map::MapLoader::load_from_file( map_file_location ) );

  infrastructure_pose.x   = declare_parameter<double>( "infrastructure_position_x", 0.0 );
  infrastructure_pose.y   = declare_parameter<double>( "infrastructure_position_y", 0.0 );
  infrastructure_pose.yaw = declare_parameter<double>( "infrastructure_yaw", 0.0 );

  max_participant_age = declare_parameter<double>( "max_participant_age", 0.5 );

  debug = declare_parameter<bool>( "debug", false );

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
DecisionMakerInfrastructure::debug_info(bool print)
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
DecisionMakerInfrastructure::traffic_participant_callback( const dynamics::TrafficParticipant& msg,
                                                           const std::string& /*currently_unused*/ )
{
  latest_traffic_participant_set.update_traffic_participants( msg );
}

void
DecisionMakerInfrastructure::update_dynamic_subscriptions()
{
  auto topic_names_and_types = get_topic_names_and_types();

  std::string topic_name    = traffic_participant_in_topic; // assumed to be set elsewhere
  std::string escaped_topic = std::regex_replace( topic_name, std::regex( R"([.^$|()\\[\]{}*+?])" ), R"(\$&)" ); // escape regex
                                                                                                                 // metacharacters

  std::regex valid_topic_regex( "^/([^/]+)/" + escaped_topic + "$" );
  // replace traffic_participant with variable
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
