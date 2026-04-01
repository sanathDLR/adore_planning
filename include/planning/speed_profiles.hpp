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

#pragma once

#include <cmath>

#include <iostream>
#include <optional>
#include <unordered_map>

#include "adore_map/map.hpp"
#include "adore_map/route.hpp"
#include "adore_math/curvature.hpp"

#include "controllers/controller.hpp"
#include "dynamics/comfort_settings.hpp"
#include "dynamics/physical_vehicle_parameters.hpp"
#include "dynamics/traffic_participant.hpp"
#include "dynamics/trajectory.hpp"
#include "dynamics/vehicle_state.hpp"
#include "planning/idm.hpp"

namespace adore
{
namespace planner
{

struct SpeedProfile
{

  using MapPointIter = std::map<double, adore::map::MapPoint>::const_iterator;

  double get_speed_at_s( double s ) const;
  double get_acc_at_s( double s ) const;

  void generate_from_route_and_participants( const map::Route& route, const dynamics::TrafficParticipantSet& traffic_participants,
                                             double initial_speed, double initial_s, double initial_time, double length );

  void backward_pass( MapPointIter& previous_it, const adore::map::Route& route, double initial_s, MapPointIter& current_it,
                      double length );

  void forward_pass( MapPointIter& it, MapPointIter& end_it, MapPointIter& prev_it, std::map<double, double>& s_to_curvature,
                     const adore::map::Route& route, const dynamics::TrafficParticipantSet& traffic_participants, double initial_time );


  SpeedProfile() {};

  void
  set_vehicle_parameters( const dynamics::PhysicalVehicleParameters& params )
  {
    vehicle_params = params;
  }

  void
  set_comfort_settings( const dynamics::ComfortSettings& settings )
  {
    comfort_settings = settings;
  }

  // Overloading the << operator
  friend std::ostream&
  operator<<( std::ostream& os, const SpeedProfile& profile )
  {
    os << "Speed Profile (s -> speed):" << std::endl;
    for( const auto& [s, speed] : profile.s_to_speed )
    {
      os << "s = " << s << " m, speed = " << speed << " m/s" << std::endl;
    }
    return os;
  }

  std::map<double, double> s_to_speed;
  std::map<double, double> s_to_acc;

private:

  std::map<double, double>                   calculate_curvature_speeds( const adore::map::Route& route, double initial_s, double length,
                                                                         double max_curvature = 0.5 );
  dynamics::PhysicalVehicleParameters        vehicle_params;
  dynamics::ComfortSettings comfort_settings;


  // default values overritten by comfort settings
  double max_acc         = 2.0;  // [m/s^2] maximum acceleration
  double max_decel       = -2.0; // [m/s^2]
  double safety_distance = 3.0;  // [m] safety distance to the nearest object
};

static adore::dynamics::Trajectory
generate_trajectory_from_speed_profile( const SpeedProfile& speed_profile, const map::Route& route,
                                        const dynamics::VehicleStateDynamic& start_state, double time_step = 0.1 )
{
  adore::dynamics::Trajectory initial_trajectory;

  double accumulated_time = 0.0;

  if( !std::isfinite( time_step ) || time_step <= 0.0 )
    time_step = 0.1;

  if( speed_profile.s_to_speed.size() < 2 )
    return initial_trajectory;

  constexpr double min_avg_speed = 0.1;

  auto it      = speed_profile.s_to_speed.begin();
  auto next_it = std::next( it );

  while( next_it != speed_profile.s_to_speed.end() )
  {
    const double s1 = it->first;
    const double s2 = next_it->first;
    const double v1 = it->second;
    const double v2 = next_it->second;

    const double delta_s = s2 - s1;

    if( delta_s <= 0.0 )
    {
      ++it;
      ++next_it;
      continue;
    }

    double avg_v = 0.5 * ( v1 + v2 );

    if( !std::isfinite( avg_v ) || avg_v < min_avg_speed )
      avg_v = min_avg_speed;

    const double delta_t = delta_s / avg_v;

    if( !std::isfinite( delta_t ) || delta_t <= 0.0 )
    {
      ++it;
      ++next_it;
      continue;
    }

    const auto pose = route.get_pose_at_s( s1 );

    adore::dynamics::VehicleStateDynamic state;
    state.x         = pose.x;
    state.y         = pose.y;
    state.yaw_angle = pose.yaw;
    state.vx        = v1;
    state.time      = accumulated_time;

    accumulated_time += delta_t;

    const double dt = accumulated_time - state.time;

    if( dt > 0.0 && std::isfinite( dt ) )
      state.ax = ( v2 - v1 ) / dt;
    else
      state.ax = 0.0;

    initial_trajectory.states.push_back( state );

    ++it;
    ++next_it;
  }

  adore::dynamics::Trajectory trajectory;

  if( initial_trajectory.states.empty() )
    return trajectory;

  const double t_final_profile = initial_trajectory.states.back().time;

  constexpr double horizon = 4.0;

  const std::size_t steps = static_cast<std::size_t>( horizon / time_step );

  dynamics::VehicleStateDynamic current_state = start_state;

  for( std::size_t i = 0; i < steps; ++i )
  {
    double t = i * time_step;

    double query_t = std::min( t, t_final_profile );

    current_state = initial_trajectory.get_state_at_time( query_t );

    current_state.time = t;

    trajectory.states.push_back( current_state );
  }

  return trajectory;
}

} // namespace planner
} // namespace adore