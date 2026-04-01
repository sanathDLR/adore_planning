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

#include "planning/analytical_planner.hpp"

#include "adore_math/distance.h"
#include "adore_math/fast_trig.h"

#include "controllers/iLQR.hpp"
#include "planning/speed_profiles.hpp"

namespace adore
{
namespace planner
{

void
RouteSamplingPlanner::set_parameters( const std::map<std::string, double>& params )
{
  for( const auto& [name, value] : params )
  {
    if( name == "max_iterations" )
      solver_params.max_iterations = value;
    if( name == "tolerance" )
      solver_params.tolerance = value;
    if( name == "max_ms" )
      solver_params.max_ms = value;
    if( name == "debug" )
      solver_params.debug = value;
  }
}

void
RouteSamplingPlanner::set_vehicle_parameters( const dynamics::PhysicalVehicleParameters& params )
{
  vehicle_params = params;
}

dynamics::PhysicalVehicleParameters
RouteSamplingPlanner::get_physical_vehicle_parameters()
{
  return vehicle_params;
}

void
RouteSamplingPlanner::set_comfort_settings( const dynamics::ComfortSettings& settings )
{
  comfort_settings = settings;
  comfort_settings.clamp( vehicle_params );
}

struct OrientedBox
{
  double x;
  double y;
  double yaw;
  double half_length;
  double half_width;
};

static bool
overlap_on_axis( const OrientedBox& a, const OrientedBox& b, double ax, double ay )
{
  // Project both boxes onto axis (ax, ay)
  auto project = [&]( const OrientedBox& o ) {
    const double c = std::cos( o.yaw );
    const double s = std::sin( o.yaw );

    const double ux = c;
    const double uy = s;
    const double vx = -s;
    const double vy = c;

    return std::abs( ( o.half_length * ux + o.half_width * vx ) * ax + ( o.half_length * uy + o.half_width * vy ) * ay );
  };

  const double dx   = b.x - a.x;
  const double dy   = b.y - a.y;
  const double dist = std::abs( dx * ax + dy * ay );

  return dist <= project( a ) + project( b );
}

static bool
boxes_intersect( const OrientedBox& a, const OrientedBox& b )
{
  const double ca = std::cos( a.yaw );
  const double sa = std::sin( a.yaw );
  const double cb = std::cos( b.yaw );
  const double sb = std::sin( b.yaw );

  // Axes: a forward, a lateral, b forward, b lateral
  return overlap_on_axis( a, b, ca, sa ) && overlap_on_axis( a, b, -sa, ca ) && overlap_on_axis( a, b, cb, sb )
      && overlap_on_axis( a, b, -sb, cb );
}

bool
RouteSamplingPlanner::is_collision_free( double x, double y, double yaw, const dynamics::TrafficParticipantSet& participants, double time,
                                         double safety_margin ) const
{
  // Ego geometry (tune if needed)
  OrientedBox ego_box;
  ego_box.x           = x;
  ego_box.y           = y;
  ego_box.yaw         = yaw;
  ego_box.half_length = 2.25 + safety_margin; // ~4.5 m / 2
  ego_box.half_width  = 1.0 + safety_margin;  // ~2.0 m / 2

  for( const auto& [id, p] : participants.participants )
  {
    dynamics::VehicleStateDynamic obj = p.state;
    if( p.trajectory.has_value() )
    {
      obj = p.trajectory.value().get_state_at_time( time );
    }

    OrientedBox obj_box;
    obj_box.x           = obj.x;
    obj_box.y           = obj.y;
    obj_box.yaw         = obj.yaw_angle;
    obj_box.half_length = 2.25; // use participant size if available
    obj_box.half_width  = 1.0;

    if( boxes_intersect( ego_box, obj_box ) )
    {
      return false;
    }
  }

  return true;
}

bool
RouteSamplingPlanner::is_lane_keep_safe( const map::Route& route, const dynamics::VehicleStateDynamic& ego,
                                         const dynamics::TrafficParticipantSet& participants ) const
{
  const double s0   = route.get_s( ego ).value();
  double       time = ego.time;

  for( double s = s0; s <= s0 + horizon_length_; s += ds_ )
  {
    auto it = route.reference_line.lower_bound( s );
    if( it == route.reference_line.end() )
      break;

    const auto   pose   = route.get_pose_at_s( s );
    const double rel_s  = s - s0;
    const double margin = ( rel_s < 5.0 ) ? 0.35 : ( rel_s < 15.0 ) ? 0.15 : 0.0;

    if( !is_collision_free( it->second.x, it->second.y, pose.yaw, participants, time, margin ) )
      if( rel_s < 10.0 )
        return false;

    time += ds_ / std::max( ego.vx, 0.5 );
  }
  return true;
}

bool
RouteSamplingPlanner::has_leading_vehicle( const map::Route& lane_route, const dynamics::VehicleStateDynamic& ego,
                                           const dynamics::TrafficParticipantSet& participants, double lookahead ) const
{
  const double s_ego = lane_route.get_s( ego ).value();

  for( const auto& [id, p] : participants.participants )
  {
    const double dx = p.state.x - ego.x;
    const double dy = p.state.y - ego.y;

    // Project onto lane direction
    const auto   pose    = lane_route.get_pose_at_s( s_ego );
    const double forward = dx * std::cos( pose.yaw ) + dy * std::sin( pose.yaw );

    // Vehicle ahead within lookahead
    if( forward > 0.0 && forward < lookahead )
      return true;
  }
  return false;
}

bool
RouteSamplingPlanner::is_inside_drivable_area( const DrivableArea& area, double s, double x, double y ) const
{
  auto it_l = area.left_boundary.lower_bound( s );
  auto it_r = area.right_boundary.lower_bound( s );

  if( it_l == area.left_boundary.end() || it_r == area.right_boundary.end() )
    return false;

  const auto& L = it_l->second;
  const auto& R = it_r->second;

  // Vector from right to left boundary
  const double vx = L.x - R.x;
  const double vy = L.y - R.y;

  // Vector from right boundary to query point
  const double wx = x - R.x;
  const double wy = y - R.y;

  const double dot  = vx * wx + vy * wy;
  const double len2 = vx * vx + vy * vy;

  if( len2 < 1e-6 )
    return false;

  const double t = dot / len2;

  // inside if projection lies between boundaries
  return ( t >= 0.0 && t <= 1.0 );
}

double
RouteSamplingPlanner::ego_time_at_s( const dynamics::VehicleStateDynamic& ego, double s_delta ) const
{
  constexpr double MIN_MOVING_SPEED = 0.5;
  constexpr double ACCEL            = 2.0; // m/s² realistic

  if( ego.vx < 0.3 )
  {
    // Ego starting from stop: use acceleration model
    // s = 0.5 * a * t²  →  t = sqrt(2s/a)
    double t = std::sqrt( 2.0 * s_delta / ACCEL );
    return ego.time + t;
  }

  // Normal moving case
  return ego.time + s_delta / std::max( ego.vx, MIN_MOVING_SPEED );
}

bool
RouteSamplingPlanner::is_overtake_clear( const map::Route& overtake_route, const dynamics::VehicleStateDynamic& ego,
                                         const dynamics::TrafficParticipantSet& participants ) const
{
  const double s_start = overtake_route.get_s( ego ).value();
  const double s_end   = s_start + horizon_length_;

  for( double s = s_start; s <= s_end; s += ds_ )
  {
    auto it = overtake_route.reference_line.lower_bound( s );
    if( it == overtake_route.reference_line.end() )
      break;

    const auto& pt   = it->second;
    const auto  pose = overtake_route.get_pose_at_s( s );

    const double ego_t = ego_time_at_s( ego, s - s_start );

    for( const auto& [id, p] : participants.participants )
    {
      dynamics::VehicleStateDynamic obj = p.state;
      if( p.trajectory )
        obj = p.trajectory->get_state_at_time( ego_t );

      OrientedBox ego_box;
      ego_box.x           = pt.x;
      ego_box.y           = pt.y;
      ego_box.yaw         = pose.yaw;
      ego_box.half_length = 2.25 + 0.35;
      ego_box.half_width  = 1.0 + 0.35;

      OrientedBox obj_box;
      obj_box.x           = obj.x;
      obj_box.y           = obj.y;
      obj_box.yaw         = obj.yaw_angle;
      obj_box.half_length = 2.25;
      obj_box.half_width  = 1.0;

      if( boxes_intersect( ego_box, obj_box ) )
        return false;

      // Optional: require time gap
      constexpr double MIN_TIME_GAP = 2.0;

      const double dist = std::hypot( obj.x - pt.x, obj.y - pt.y );

      const double obj_speed = std::max( std::hypot( obj.vx, obj.vy ), 0.1 );

      const double obj_arrival = obj.time + dist / obj_speed;

      if( std::abs( obj_arrival - ego_t ) < MIN_TIME_GAP )
        return false;
    }
  }

  return true;
}

bool
RouteSamplingPlanner::has_static_leading_vehicle( const map::Route& lane_route, const dynamics::VehicleStateDynamic& ego,
                                                  const dynamics::TrafficParticipantSet& participants, double lookahead,
                                                  double static_speed_threshold ) const
{
  const double s_ego = lane_route.get_s( ego ).value();

  for( const auto& [id, p] : participants.participants )
  {
    const double speed = std::hypot( p.state.vx, p.state.vy );

    // Only consider nearly static objects
    if( speed > static_speed_threshold )
      continue;

    const double obj_s = lane_route.get_s( p.state ).value();
    const double ds    = obj_s - s_ego;

    if( ds > 0.0 && ds < lookahead )
      return true;
  }

  return false;
}

void
RouteSamplingPlanner::update_maneuver_state( const map::Route& lane_route, const map::Route& overtake_route,
                                             const dynamics::VehicleStateDynamic& ego, const dynamics::TrafficParticipantSet& participants )
{
  const double s = lane_route.get_s( ego ).value();

  switch( maneuver_state_ )
  {
    case ManeuverState::LaneKeep:
    {
      constexpr double STATIC_THRESHOLD = 0.1; // m/s

      const bool static_blocking = has_static_leading_vehicle( lane_route, ego, participants, 40.0, STATIC_THRESHOLD );

      const bool lane_blocked = !is_lane_keep_safe( lane_route, ego, participants );

      const bool overtake_clear = is_overtake_clear( overtake_route, ego, participants );
      // const bool overtake_clear = is_route_safe_over_horizon( overtake_route, ego, participants, 60, 0.5 );

      if( lane_blocked && static_blocking && overtake_clear )
        maneuver_state_ = ManeuverState::Overtake;

      break;
    }

    case ManeuverState::Overtake:
    {
      const bool lane_safe = is_lane_keep_safe( lane_route, ego, participants );

      const bool obstacle_passed = !has_leading_vehicle( lane_route, ego, participants, 40.0 );

      if( lane_safe && obstacle_passed )
      {
        maneuver_state_ = ManeuverState::ReturnToLane;
        return_start_s_ = s;
      }
      break;
    }

    case ManeuverState::ReturnToLane:
    {
      if( s - return_start_s_ > RETURN_LENGTH )
        maneuver_state_ = ManeuverState::LaneKeep;
      break;
    }
  }
}

static double
smoothstep( double x )
{
  x = std::clamp( x, 0.0, 1.0 );
  return 1.0 - ( 3 * x * x - 2 * x * x * x );
}

map::Route
RouteSamplingPlanner::build_return_route( const map::Route& overtake_route, const map::Route& lane_route ) const
{
  map::Route result = overtake_route;

  for( auto& [s, pt] : result.reference_line )
  {
    if( s < return_start_s_ )
      continue;

    const auto   ref = lane_route.get_pose_at_s( s );
    const double nx  = -std::sin( ref.yaw );
    const double ny  = std::cos( ref.yaw );

    const double dx = pt.x - ref.x;
    const double dy = pt.y - ref.y;
    const double d  = dx * nx + dy * ny;

    const double w = smoothstep( ( s - return_start_s_ ) / RETURN_LENGTH );

    pt.x = ref.x + w * d * nx;
    pt.y = ref.y + w * d * ny;
  }
  return result;
}

bool
RouteSamplingPlanner::is_route_safe_over_horizon( const map::Route& route, const dynamics::VehicleStateDynamic& ego,
                                                  const dynamics::TrafficParticipantSet& participants, double horizon_length,
                                                  double ds ) const
{
  const double s_start = route.get_s( ego ).value();
  const double s_end   = s_start + horizon_length;

  double time = ego.time;

  for( double s = s_start; s <= s_end; s += ds )
  {
    auto it = route.reference_line.lower_bound( s );
    if( it == route.reference_line.end() )
      break;

    const auto& pt = it->second;

    // Use reference yaw for stability
    const auto   ref_pose = route.get_pose_at_s( s );
    const double yaw      = ref_pose.yaw;

    // Be strict near ego, permissive further away
    const double rel_s         = s - s_start;
    const double safety_margin = ( rel_s < 5.0 ) ? 0.35 : ( rel_s < 15.0 ) ? 0.15 : 0.0;

    if( !is_collision_free( pt.x, pt.y, yaw, participants, time, safety_margin ) )
    {
      // Only reject if conflict is near-term
      if( rel_s < 10.0 )
        return false;
    }

    time = ego_time_at_s(ego, s - s_start);
  }

  return true;
}

map::Route
RouteSamplingPlanner::sample_route( const map::Route& lane_route, const DrivableArea& drivable_area,
                                    const dynamics::VehicleStateDynamic& ego, const dynamics::TrafficParticipantSet& participants )
{
  // ----- Build overtake candidate -----
  map::Route overtake = lane_route;

  const double s0 = lane_route.get_s( ego ).value();
  const double s1 = s0 + horizon_length_;

  auto& ref = overtake.reference_line;
  for( auto it = ref.lower_bound( s0 ); it != ref.lower_bound( s1 ); )
    it = ref.erase( it );

  double time = ego.time;

  for( std::size_t i = 0; i < horizon_length_ / ds_; ++i )
  {
    const double s_global = s0 + i * ds_;

    const auto pose = lane_route.get_pose_at_s( s_global );

    const double c   = math::fast_cos( pose.yaw );
    const double sng = math::fast_sin( pose.yaw );

    const double nx = -sng;
    const double ny = c;

    bool found = false;

    constexpr double lane_half_width   = 2.5;
    const int        max_lateral_steps = static_cast<int>( lane_half_width / dd_ );

    for( int k = 0; k <= max_lateral_steps && !found; ++k )
    {
      for( int sign : { +1 } ) // Only left overtake allowed now
      {
        if( k == 0 && sign == -1 )
          continue;

        const double d = sign * k * dd_;
        if( std::abs( d ) > lane_half_width )
          continue;

        const double x            = pose.x + d * nx;
        const double y            = pose.y + d * ny;
        const double preview_s    = 5.0; // meters ahead
        const double preview_time = time + preview_s / std::max( ego.vx, 0.5 );

        if( is_inside_drivable_area( drivable_area, s_global, x, y )
            && is_collision_free( x, y, pose.yaw, participants, preview_time, 0.35 ) )
        {
          adore::map::MapPoint pt;
          pt.x                              = x;
          pt.y                              = y;
          overtake.reference_line[s_global] = pt;

          found = true;
          break;
        }
      }
    }
    time += ds_ / std::max( ego.vx, 0.5 );
  }

  // ----- FSM -----
  update_maneuver_state( lane_route, overtake, ego, participants );

  // ----- Select route -----
  if( maneuver_state_ == ManeuverState::Overtake )
    return get_smooth_route( overtake, lane_route );

  if( maneuver_state_ == ManeuverState::ReturnToLane )
    return build_return_route( overtake, lane_route );

  return lane_route;
}

map::Route
RouteSamplingPlanner::get_smooth_route( const map::Route& raw_route, const map::Route& reference_route ) const
{
  if( raw_route.reference_line.size() < 5 )
    return raw_route;

  // --- Extract raw points ---
  std::vector<double>               s_vals;
  std::vector<adore::map::MapPoint> pts;

  for( const auto& [s, p] : raw_route.reference_line )
  {
    s_vals.push_back( s );
    pts.push_back( p );
  }

  const size_t N = pts.size();

  // --- Compute lateral offsets w.r.t reference route ---
  std::vector<double> d( N );

  for( size_t i = 0; i < N; ++i )
  {
    const auto ref_pose = reference_route.get_pose_at_s( s_vals[i] );

    const double nx = -std::sin( ref_pose.yaw );
    const double ny = std::cos( ref_pose.yaw );

    const double dx = pts[i].x - ref_pose.x;
    const double dy = pts[i].y - ref_pose.y;

    d[i] = dx * nx + dy * ny;
  }

  // --- Smooth lateral offsets only ---
  std::vector<double> d_smoothed = d;

  for( int iter = 0; iter < 10; ++iter )
  {
    for( size_t i = 1; i + 1 < N; ++i )
    {
      d_smoothed[i] = 0.25 * d_smoothed[i - 1] + 0.50 * d_smoothed[i] + 0.25 * d_smoothed[i + 1];
    }
  }

  // --- Reconstruct Cartesian points ---
  map::Route smoothed = raw_route;

  for( size_t i = 0; i < N; ++i )
  {
    const auto ref_pose = reference_route.get_pose_at_s( s_vals[i] );

    const double nx = -std::sin( ref_pose.yaw );
    const double ny = std::cos( ref_pose.yaw );

    adore::map::MapPoint p;
    p.x = ref_pose.x + d_smoothed[i] * nx;
    p.y = ref_pose.y + d_smoothed[i] * ny;

    smoothed.reference_line[s_vals[i]] = p;
  }

  return smoothed;
}

double
project_object_to_route_s( const map::Route& route, const dynamics::VehicleStateDynamic& obj )
{
  // Use closest s on route (simplified)
  return route.get_s( obj ).value();
}

double
RouteSamplingPlanner::compute_ego_target_speed( const map::Route& lane_route, const dynamics::VehicleStateDynamic& ego,
                                                const dynamics::TrafficParticipantSet& participants )
{
  constexpr double MAX_DECEL    = 3.0;
  constexpr double MIN_TIME_GAP = 2.0;

  constexpr double MERGE_DIST        = 20.0;
  constexpr double MERGE_STOP_DIST   = 4.0;
  constexpr double ARRIVAL_MARGIN    = 1.5;
  constexpr double MERGE_COMMIT_TIME = 4.0;

  const double ego_s0 = lane_route.get_s( ego ).value();
  const double ego_v0 = std::max( ego.vx, 0.1 );

  // --- if merge already decided ---
  if( merge_state_ != MergeState::NONE )
  {
    if( ego.time < merge_release_time_ )
    {
      if( merge_state_ == MergeState::EGO_YIELD )
        return 0.0;

      if( merge_state_ == MergeState::EGO_GO )
        return ego_v0; // do NOT reconsider merge
    }

    // merge finished
    merge_state_ = MergeState::NONE;
  }

  double target_speed = ego_v0;

  for( const auto& [id, p] : participants.participants )
  {
    dynamics::VehicleStateDynamic obj = p.state;

    if( p.trajectory )
      obj = p.trajectory->get_state_at_time( ego.time );

    double dx = obj.x - ego.x;
    double dy = obj.y - ego.y;

    double dist = std::hypot( dx, dy );

    if( dist > MERGE_DIST )
      continue;

    double forward = dx * std::cos( ego.yaw_angle ) + dy * std::sin( ego.yaw_angle );

    if( forward < -5.0 )
      continue;

    double obj_s = project_object_to_route_s( lane_route, obj );

    double ds = obj_s - ego_s0;

    if( ds < -2.0 )
      continue;

    double obj_speed = std::max( std::hypot( obj.vx, obj.vy ), 0.1 );

    double ego_arrival = ds / std::max( ego_v0, 0.1 );

    double obj_arrival = dist / obj_speed;
    double obj_ds      = obj_s - ego_s0;
    obj_arrival        = obj_ds / obj_speed;

    if( std::abs( ego_arrival - obj_arrival ) < ARRIVAL_MARGIN )
    {
      if( obj_arrival < ego_arrival )
      {
        merge_state_        = MergeState::EGO_YIELD;
        merge_release_time_ = ego.time + MERGE_COMMIT_TIME;

        if( ds < MERGE_STOP_DIST )
          return 0.0;

        double allowed_speed = ( ds - MERGE_STOP_DIST ) / std::max( ego_arrival, 0.1 );

        target_speed = std::min( target_speed, allowed_speed );
      }
      else
      {
        merge_state_        = MergeState::EGO_GO;
        merge_release_time_ = ego.time + MERGE_COMMIT_TIME;
      }
    }

    if( ds > 0.0 )
    {
      double allowed_speed = ds / ( ego_arrival + MIN_TIME_GAP );

      target_speed = std::min( target_speed, allowed_speed );
    }
  }

  double max_speed_drop = MAX_DECEL * ds_;

  target_speed = std::max( 0.0, std::max( ego_v0 - max_speed_drop, target_speed ) );

  if( target_speed < 0.3 )
    target_speed = 0.0;

  return target_speed;
}

mas::MotionModel
RouteSamplingPlanner::get_planning_model( const dynamics::PhysicalVehicleParameters& params )
{
  return [params]( const mas::State& x, const mas::Control& u ) -> mas::StateDerivative {
    mas::StateDerivative dxdt;
    dxdt.setZero( 5 );
    dxdt( 0 ) = x( 3 ) * std::cos( x( 2 ) );                    // x
    dxdt( 1 ) = x( 3 ) * std::sin( x( 2 ) );                    // y
    dxdt( 2 ) = x( 3 ) * std::tan( x( 4 ) ) / params.wheelbase; // yaw_angle
    dxdt( 3 ) = u( 1 );                                         // v
    dxdt( 4 ) = u( 0 );                                         // delta
    return dxdt;
  };
}

mas::StageCostFunction
RouteSamplingPlanner::make_trajectory_cost( const dynamics::Trajectory& ref_traj )
{
  return [this, &ref_traj]( const mas::State& x, const mas::Control& u, std::size_t k ) -> double {
    double cost = 0.0;

    const double t   = k * dt;
    const auto   ref = ref_traj.get_state_at_time( t );

    const double dx = x( 0 ) - ref.x;
    const double dy = x( 1 ) - ref.y;

    const double c = math::fast_cos( ref.yaw_angle );
    const double s = math::fast_sin( ref.yaw_angle );

    const double lon_err = dx * c + dy * s;
    const double lat_err = -dx * s + dy * c;
    const double hdg_err = math::normalize_angle( x( 2 ) - ref.yaw_angle );
    const double spd_err = x( 3 ) - std::min( ref.vx, ego_interaction_speed );

    cost += weights.lane_error * lat_err * lat_err;
    cost += weights.long_error * lon_err * lon_err;
    cost += weights.heading_error * hdg_err * hdg_err;
    cost += weights.speed_error * spd_err * spd_err;
    cost += weights.steering_angle * u( 0 ) * u( 0 ) + weights.acceleration * u( 1 ) * u( 1 );
    return cost;
  };
}

PlannerResult
RouteSamplingPlanner::plan_route_trajectory( const map::Route& latest_route, const dynamics::VehicleStateDynamic& current_state,
                                             const dynamics::TrafficParticipantSet& traffic_participants )
{
  PlannerResult planner_output;
  auto          modified_route = latest_route;
  const double  state_s        = latest_route.get_s( current_state ).value();
  const double  da_start_s     = state_s - config.drivable_area_before;
  const double  da_end_s       = state_s + config.drivable_area_length;

  // 1. Predict participants
  // We make a local copy because prediction modifies the trajectories in place
  auto prediction_participants = traffic_participants;
  // participant_predictor.plan_trajectories( prediction_participants );

  // 2. Dynamically select lane scope based on oncoming traffic
  if( has_traffic_in_oncoming_lanes( latest_route, da_start_s, da_end_s, prediction_participants ) )
  {
    config.da_config.lane_scope = LaneScope::SameDirection;
  }
  else
  {
    config.da_config.lane_scope = LaneScope::AllLanes;
  }

  auto drivable_area = create_drivable_area( latest_route, da_start_s, da_end_s, prediction_participants, vehicle_params,
                                             config.da_config );

  planner_output.drivable_area = drivable_area;

  if( overtaking_allowed )
    modified_route = sample_route( latest_route, drivable_area, current_state, traffic_participants );
  double initial_s = modified_route.get_s( current_state ).value();

  SpeedProfile speed_profile;
  speed_profile.set_vehicle_parameters( vehicle_params );
  speed_profile.set_comfort_settings( comfort_settings );

  speed_profile.generate_from_route_and_participants( modified_route, traffic_participants, current_state.vx, initial_s, current_state.time,
                                                      100. );

  auto ref_traj = generate_trajectory_from_speed_profile( speed_profile, modified_route, current_state, dt );

  // PID-based initial guess
  controllers::PurePursuit pid;
  pid.model        = dynamics::PhysicalVehicleModel();
  pid.model.params = vehicle_params;

  auto guess_state = current_state;
  guess_state.time = 0.0;

  dynamics::Trajectory initial_guess;

  for( size_t i = 0; i < horizon_steps; ++i )
  {
    auto command               = pid.get_next_vehicle_command( ref_traj, guess_state );
    guess_state.ax             = command.acceleration;
    guess_state.steering_angle = command.steering_angle;
    initial_guess.states.push_back( guess_state );
    guess_state = dynamics::integrate_rk4( guess_state, command, dt, pid.model.motion_model );
  }
  ego_interaction_speed = compute_ego_target_speed( modified_route, current_state, traffic_participants );
  // std::cerr << "ref_speed: " << ref_traj.states[0].vx << " interaction_speed: " << ego_interaction_speed << std::endl;
  dynamics::Trajectory out_traj = optimize_trajectory( current_state, ref_traj, initial_guess );
  planner_output.trajectory     = out_traj;
  planner_output.modified_route = modified_route;
  return planner_output;
}

dynamics::Trajectory
RouteSamplingPlanner::optimize_trajectory( const dynamics::VehicleStateDynamic& current_state, const dynamics::Trajectory& ref_traj,
                                           const dynamics::Trajectory& initial_guess )
{
  start_state          = current_state;
  reference_trajectory = ref_traj;
  guess_trajectory     = initial_guess;
  setup_problem();
  solve_problem();
  auto out_trajectory = extract_trajectory();
  if( problem->best_cost > 30.0 && current_state.vx > 2.0 && counter < 15 )
  {
    counter++;
    return previous_trajectory;
  }
  counter             = 0;
  previous_trajectory = out_trajectory;
  return out_trajectory;
}

void
RouteSamplingPlanner::solve_problem()
{
  mas::SolverParams params;
  params["max_iterations"] = solver_params.max_iterations;
  params["tolerance"]      = solver_params.tolerance;
  params["max_ms"]         = solver_params.max_ms;
  params["debug"]          = solver_params.debug;

  auto solve_with = [&]( auto&& solver, double max_ms ) {
    params["max_ms"] = max_ms;
    solver.set_params( params );
    solver.solve( *problem );
    problem->update_initial_with_best();
  };

  // first pass with collocation
  solve_with( mas::OSQPCollocation{}, 60 );
}

dynamics::Trajectory
RouteSamplingPlanner::extract_trajectory()
{
  dynamics::Trajectory trajectory;
  trajectory.states.reserve( problem->horizon_steps );
  for( size_t i = 0; i < problem->horizon_steps; ++i )
  {
    dynamics::VehicleStateDynamic state;
    auto                          x = problem->best_states.col( i );
    auto                          u = problem->best_controls.col( i );

    state.x              = x( 0 );
    state.y              = x( 1 );
    state.yaw_angle      = math::normalize_angle( x( 2 ) );
    state.vx             = x( 3 );
    state.time           = start_state.time + i * 0.1;
    state.steering_angle = math::normalize_angle( x( 4 ) );
    state.ax             = u( 1 );

    trajectory.states.push_back( state );
  }

  return trajectory;
}

void
RouteSamplingPlanner::setup_problem()
{
  problem = std::make_shared<mas::OCP>();

  problem->state_dim     = 5;
  problem->control_dim   = 2;
  problem->horizon_steps = 40;
  problem->dt            = dt;
  problem->initial_state = Eigen::VectorXd( 5 );
  problem->dynamics      = get_planning_model( vehicle_params );


  Eigen::VectorXd lower_bounds( problem->control_dim ), upper_bounds( problem->control_dim );
  lower_bounds << -1.0, vehicle_params.acceleration_min;
  upper_bounds << 1.0, vehicle_params.acceleration_max;
  problem->input_lower_bounds = lower_bounds;
  problem->input_upper_bounds = upper_bounds;
  problem->stage_cost         = make_trajectory_cost( reference_trajectory );

  problem->initial_state << start_state.x, start_state.y, start_state.yaw_angle, start_state.vx, start_state.steering_angle;

  // initialize best guess controls from guess trajectory
  problem->initial_controls = mas::ControlTrajectory::Zero( problem->control_dim, problem->horizon_steps );
  for( size_t i = 0; i < problem->horizon_steps; ++i )
  {
    if( i >= guess_trajectory.states.size() )
      break;
    double t                          = i * dt;
    auto   ref                        = guess_trajectory.get_state_at_time( t );
    problem->initial_controls( 1, i ) = ref.ax;             // acceleration
    problem->initial_controls( 0, i ) = 0.0; // steering rate
  }

  problem->initialize_problem();
  problem->verify_problem();
}

} // namespace planner
} // namespace adore