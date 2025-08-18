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

#include "decision_maker_infrastructure.hpp"

#include <adore_dynamics_conversions.hpp>
#include <adore_math/point.h>
#include <adore_math/polygon.h>
#include "std_msgs/msg/string.hpp"

namespace adore
{

DecisionMakerInfrastructure::DecisionMakerInfrastructure(const rclcpp::NodeOptions & options) : Node( "decision_maker_infrastructure" , options)
{
  load_parameters();
  // Load map
  road_map = map::MapLoader::load_from_file( map_file_location );
  create_subscribers();
  create_publishers();
  print_init_info();
}

void
DecisionMakerInfrastructure::run()
{
  overview = "";

  // auto start_time = std::chrono::high_resolution_clock::now(); // Start timer
  debug_info(debug_mode_active);

  if( road_map.has_value() )
  {
    compute_routes_for_traffic_participant_set( latest_traffic_participant_set, road_map.value() );
  }
  all_vehicles_follow_routes();
  // auto end_time = std::chrono::high_resolution_clock::now(); // End timer

  // Compute elapsed time in milliseconds
  // double elapsed_time_ms = std::chrono::duration<double, std::milli>( end_time - start_time ).count();

  // Log the elapsed time
  // RCLCPP_INFO( this->get_logger(), "Planning took %.3f ms", elapsed_time_ms );

  publish_local_map();
  publish_infrastructure_position(); 
  publish_infrastructure_info();

  latest_traffic_participant_set.remove_old_participants(1.0, now().seconds());

  std_msgs::msg::String overview_msg;
  overview_msg.data = overview;
  publisher_overview->publish(overview_msg);
}

void
DecisionMakerInfrastructure::all_vehicles_follow_routes()
{
  double time_start = now().seconds();
  auto mrm_participant_set = latest_traffic_participant_set;
  for( auto& [id, participant] : latest_traffic_participant_set.participants )
  {
    if ( participant.route.has_value() )
    {
      for( auto& p : participant.route.value().center_lane )
      {
        if( std::any_of( stopping_points.begin(), stopping_points.end(),
                        [&]( const auto& s ) { return adore::math::distance_2d( s, p.second ) < 3.0; } ) )
        {
          p.second.max_speed = 0;
        }
        else
        {
          p.second.max_speed = std::nullopt;
        }
      }
    }
  }
  multi_agent_PID_planner.plan_trajectories( latest_traffic_participant_set );
  multi_agent_PID_planner_MRM.plan_trajectories( mrm_participant_set );
  for( auto& [id, participant] : latest_traffic_participant_set.participants )
  {
    if( mrm_participant_set.participants.at( id ).trajectory.has_value() )
      participant.mrm_trajectory = mrm_participant_set.participants.at( id ).trajectory.value();
  }

  double time_used_to_calculate_trajectory = now().seconds() - time_start;
  overview += "time used to calculate trajectories: " + std::to_string(time_used_to_calculate_trajectory) + ", ";

  auto traffic_participant_set_msg = dynamics::conversions::to_ros_msg( latest_traffic_participant_set );
  traffic_participant_set_msg.header.frame_id = "world";

  publisher_planned_traffic->publish( traffic_participant_set_msg );
}

void
DecisionMakerInfrastructure::create_subscribers()
{
  subscriber_traffic_participant_set = create_subscription<adore_ros2_msgs::msg::TrafficParticipantSet>(
    "traffic_participants", 1, std::bind( &DecisionMakerInfrastructure::traffic_participants_callback, this, std::placeholders::_1 ) );
  subscriber_traffic_signals = create_subscription<adore_ros2_msgs::msg::TrafficSignals>(
    "/global/traffic_signals", 1, std::bind( &DecisionMakerInfrastructure::traffic_signals_callback, this, std::placeholders::_1 ) );
  main_timer = create_wall_timer( 100ms, std::bind( &DecisionMakerInfrastructure::run, this ) );
}

void
DecisionMakerInfrastructure::create_publishers()
{
  publisher_infrastructure_info = create_publisher<adore_ros2_msgs::msg::InfrastructureInfo>( "infrastructure_info", 10 );
  publisher_planned_traffic = create_publisher<adore_ros2_msgs::msg::TrafficParticipantSet>( "traffic_participants_with_trajectories", 1 );
  publisher_local_map       = create_publisher<adore_ros2_msgs::msg::Map>( "local_map", 1 );
  publisher_infrastructure_position = create_publisher<adore_ros2_msgs::msg::VisualizableObject>( "infrastructure_position", 1 );
  publisher_overview = create_publisher<std_msgs::msg::String>( "overview", 1 );
}

void
DecisionMakerInfrastructure::load_parameters()
{
  declare_parameter( "debug_mode_active", true );
  get_parameter( "debug_mode_active", debug_mode_active );

  declare_parameter( "dt", 0.1 );
  get_parameter( "dt", dt );

  declare_parameter( "max_acceleration", 2.0 );
  declare_parameter( "min_acceleration", -2.0 );
  declare_parameter( "max_steering", 0.7 );
  get_parameter( "max_acceleration", command_limits.max_acceleration );
  get_parameter( "min_acceleration", command_limits.min_acceleration );
  get_parameter( "max_steering", command_limits.max_steering_angle );

  declare_parameter( "map file", "" );
  get_parameter( "map file", map_file_location );

  declare_parameter( "infrastructure_position_x", 0.0 );
  declare_parameter( "infrastructure_position_y", 0.0 );
  declare_parameter( "infrastructure_yaw", 0.0 );
  get_parameter( "infrastructure_position_x", infrastructure_pose.x );
  get_parameter( "infrastructure_position_y", infrastructure_pose.y );
  get_parameter( "infrastructure_yaw", infrastructure_pose.yaw );

  // Multi Agent PID related parameters
  std::vector<std::string> keys;
  std::vector<double>      values;
  declare_parameter( "multi_agent_PID_settings_keys", keys );
  declare_parameter( "multi_agent_PID_settings_values", values );
  get_parameter( "multi_agent_PID_settings_keys", keys );
  get_parameter( "multi_agent_PID_settings_values", values );

  if( keys.size() != values.size() )
  {
    RCLCPP_ERROR( this->get_logger(), "multi agent PID settings keys and values size mismatch!" );
    return;
  }
  for( size_t i = 0; i < keys.size(); ++i )
  {
    multi_agent_PID_settings.insert( { keys[i], values[i] } );
    std::cerr << "keys: " << keys[i] << ": " << values[i] << std::endl;
  }


  std::vector<double> validity_area_points; // request assistance polygon
  declare_parameter( "validity_polygon", std::vector<double>{} );
  get_parameter( "validity_polygon", validity_area_points );

  // Convert the parameter into a Polygon2d
  if( validity_area_points.size() >= 6 ) // minimum 3 x, 3 y
  {
    math::Polygon2d validity_area;
    validity_area.points.reserve( validity_area_points.size() / 2 );

    for( size_t i = 0; i < validity_area_points.size(); i += 2 )
    {
      double x = validity_area_points[i];
      double y = validity_area_points[i + 1];
      validity_area.points.push_back( { x, y } );
    }
    latest_traffic_participant_set.validity_area = validity_area;
  }

  multi_agent_PID_planner.set_parameters( multi_agent_PID_settings );
  multi_agent_PID_planner_MRM.max_allowed_speed = 0.0;
}

void
DecisionMakerInfrastructure::print_init_info()
{
  std::cout << "DecisionMakerInfrastructure node initialized.\n";
  std::cout << "Debug mode: " << ( debug_mode_active ? "Active" : "Inactive" ) << std::endl;
}

void
DecisionMakerInfrastructure::debug_info(bool print)
{
  if ( !road_map.has_value() )
  {
    overview += "Missing map, ";
  }

  if ( !print ) return;

  double current_time_seconds = now().seconds();
  std::cerr << "------- Decision Maker Infrastructure Debug Information -------" << std::endl;
  std::cerr << "Current Time: " << current_time_seconds << " seconds" << std::endl;

  if( road_map.has_value() )
    std::cerr << "Local map data available.\n";
  else
    std::cerr << "No local map data.\n";

  std::cerr << "traffic participants " << latest_traffic_participant_set.participants.size() << std::endl;

  std::cerr << "------- ============================== -------" << std::endl;
}

void
DecisionMakerInfrastructure::compute_routes_for_traffic_participant_set( dynamics::TrafficParticipantSet& traffic_participant_set,
                                                                         const map::Map&                  road_map )
{
  double time_start = now().seconds();
  int controlable_participants = 0;
  int controlable_participants_inside_validity_area = 0;

  std::string text_to_add_later;
  
  for( auto& [id, participant] : traffic_participant_set.participants )
  {
    bool has_goal  = participant.goal_point.has_value();
    bool has_route = participant.route.has_value();

    bool has_own_map = false;
    if ( has_route )
    {
      has_own_map = participant.route.value().map !=  nullptr;
      if ( !has_own_map )
      {
        overview += "participant " + std::to_string(id) + " with route has no map, ";
      }
    }

    if( has_goal && !has_route )
    {
      participant.route = map::Route( participant.state, participant.goal_point.value(), road_map );
      if( participant.route->center_lane.empty() )
      {
        participant.route = std::nullopt;
        std::cerr << "No route found for traffic participant " << participant.id << std::endl;
        text_to_add_later += "and cannot find route for participant " + std::to_string(id);
      }
    }

    // This is only uses for debugging
    if( has_goal )
    {
      // This is all used for overview purposes, move it somewhere else
      controlable_participants++;


      if (traffic_participant_set.validity_area.has_value()) // @TODO, this filter needs to be done elsewhere.
      {
        math::Point2d participant_position = { participant.state.x, participant.state.y };
        if (traffic_participant_set.validity_area.value().point_inside(participant_position))
        {
          controlable_participants_inside_validity_area++;
        }
      }
      else
      {
        controlable_participants_inside_validity_area++;
      }
      // ------------------------------------------
      
    }
  }

  overview += "observing " + std::to_string(traffic_participant_set.participants.size()) + " participants, ";
  overview += "trajectory planning for " + std::to_string(controlable_participants) + " participants, ";
  overview += "with " + std::to_string(controlable_participants_inside_validity_area) + " inside the validty area, ";
  overview += text_to_add_later;

  double time_used_calculating_route = now().seconds() - time_start;
  overview += "route calculation time: " + std::to_string(time_used_calculating_route) + ", ";
}

void
DecisionMakerInfrastructure::publish_local_map()
{
  if( !road_map.has_value() )
    return;
  local_map = road_map->get_submap( infrastructure_pose, local_map_size, local_map_size );
  publisher_local_map->publish( map::conversions::to_ros_msg( local_map.value() ) );
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
}

void
DecisionMakerInfrastructure::publish_infrastructure_info()
{
  
  adore_ros2_msgs::msg::InfrastructureInfo infrastructure_info_msg;
  infrastructure_info_msg.position_x = infrastructure_pose.x;
  infrastructure_info_msg.position_y = infrastructure_pose.y;
  infrastructure_info_msg.yaw = infrastructure_pose.yaw;

  adore_ros2_msgs::msg::Polygon2d validity_area_msg;
  if ( latest_traffic_participant_set.validity_area.has_value() )
  {
    for (auto point : latest_traffic_participant_set.validity_area.value().points)
    {
        adore_ros2_msgs::msg::Point2d point_msg;
        point_msg.x = point.x;
        point_msg.y = point.y;

        validity_area_msg.points.push_back(point_msg);
    }
    infrastructure_info_msg.validity_area = validity_area_msg;
  }
  publisher_infrastructure_info->publish(infrastructure_info_msg);
}

void
DecisionMakerInfrastructure::traffic_signals_callback( const adore_ros2_msgs::msg::TrafficSignals& msg )
{
  stopping_points.clear();
  for( const auto& signal : msg.signals )
  {
    if ( signal.state == 0 )
    {
      stopping_points.emplace_back( signal.x, signal.y );
    }
  }
}

void
DecisionMakerInfrastructure::traffic_participants_callback( const adore_ros2_msgs::msg::TrafficParticipantSet& msg )
{
  // auto validity_area = latest_traffic_participant_set.validity_area;
  // latest_traffic_participant_set = dynamics::conversions::to_cpp_type( msg );// @TODO, this is temporary, add back the uncommented area below
  // latest_traffic_participant_set.validity_area = validity_area;
  // std::cerr << "participant received - decision maker infra " << std::endl;
  auto participants_cpp = dynamics::conversions::to_cpp_type( msg );
  for ( auto& [id, participant] : participants_cpp.participants )
  {
    // if ( !road_map.has_value() && participant.route.has_value())
    // {
    //   par
  
    // }
    //
    // if ( !latest_traffic_participant_set.participants.contains(id) )
    // {
      participant.route = std::nullopt;
    // }

    latest_traffic_participant_set.update_traffic_participants(participant); // @TODO, investigate this more, as it does not add them unless their are inside validity area
  }
}

}; // namespace adore


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(adore::DecisionMakerInfrastructure)
