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

#include <vector>

#include "adore_map/map_point.hpp"
#include "adore_map/route.hpp"

#include "dynamics/comfort_settings.hpp"
#include "dynamics/traffic_participant.hpp"
#include "dynamics/trajectory.hpp"
#include "dynamics/vehicle_state.hpp"
#include "multi_agent_solver/multi_agent_solver.hpp"
#include "planning/common/planning_config.hpp"
#include "planning/drivable_area.hpp"
#include "planning/speed_profiles.hpp"

namespace adore
{
namespace planner
{

struct MotionPlannerConfig
{
  DrivableAreaConfig da_config;
  SpeedProfileConfig sp_config;

  // General planning settings can be added here
  double drivable_area_length = 100.0;
  double drivable_area_before = 10.0;
};

struct PlannerResult
{
  std::optional<dynamics::Trajectory> trajectory;
  std::optional<DrivableArea>         drivable_area; // For debugging/visualization
  bool                                success = false;
  std::string                         message;
  std::optional<map::Route>           modified_route;
};

class RouteSamplingPlanner
{
public:

  RouteSamplingPlanner() = default;

  void set_parameters( const std::map<std::string, double>& params );
  void set_vehicle_parameters( const dynamics::PhysicalVehicleParameters& params );
  void set_comfort_settings( const dynamics::ComfortSettings& settings );

  dynamics::PhysicalVehicleParameters get_physical_vehicle_parameters();

  map::Route sample_route( const map::Route& route, const DrivableArea& drivable_area, const dynamics::VehicleStateDynamic& ego,
                           const dynamics::TrafficParticipantSet& participants );

  map::Route get_smooth_route( const map::Route& raw_route, const map::Route& original_route ) const;

  PlannerResult plan_route_trajectory( const map::Route& latest_route, const dynamics::VehicleStateDynamic& current_state,
                                       const dynamics::TrafficParticipantSet& traffic_participants );

  dynamics::Trajectory optimize_trajectory( const dynamics::VehicleStateDynamic& current_state,
                                            const dynamics::Trajectory&          reference_trajectory,
                                            const dynamics::Trajectory&          initial_guess = dynamics::Trajectory() );

private:

  bool is_collision_free( double x, double y, double yaw, const dynamics::TrafficParticipantSet& participants, double time,
                          double safety_margin ) const;
  bool is_route_safe_over_horizon( const map::Route& route, const dynamics::VehicleStateDynamic& ego,
                                   const dynamics::TrafficParticipantSet& participants, double horizon_length, double ds ) const;
  void update_maneuver_state( const map::Route& lane_route, const map::Route& overtake_candidate, const dynamics::VehicleStateDynamic& ego,
                              const dynamics::TrafficParticipantSet& participants );
  bool has_static_leading_vehicle( const map::Route& lane_route, const dynamics::VehicleStateDynamic& ego,
                                   const dynamics::TrafficParticipantSet& participants, double lookahead,
                                   double static_speed_threshold ) const;
  bool is_inside_drivable_area( const DrivableArea& area, double s, double x, double y ) const;
  bool is_overtake_clear( const map::Route& overtake_route, const dynamics::VehicleStateDynamic& ego,
                          const dynamics::TrafficParticipantSet& participants ) const;
  bool is_lane_keep_safe( const map::Route& lane_route, const dynamics::VehicleStateDynamic& ego,
                          const dynamics::TrafficParticipantSet& participants ) const;
  bool has_leading_vehicle( const map::Route& lane_route, const dynamics::VehicleStateDynamic& ego,
                            const dynamics::TrafficParticipantSet& participants, double lookahead ) const;

  double     compute_ego_target_speed( const map::Route& lane_route, const dynamics::VehicleStateDynamic& ego,
                                       const dynamics::TrafficParticipantSet& participants );
  double     ego_time_at_s( const dynamics::VehicleStateDynamic& ego, double s_delta ) const;
  map::Route build_return_route( const map::Route& overtake_route, const map::Route& lane_route ) const;
  void       setup_problem();

  mas::StageCostFunction        make_trajectory_cost( const dynamics::Trajectory& ref_traj );
  mas::MotionModel              get_planning_model( const dynamics::PhysicalVehicleParameters& params );
  dynamics::Trajectory          extract_trajectory();
  void                          solve_problem();
  double                        dt                  = 0.1;
  double                        conflict_lock_until = 0.0;
  bool                          ego_is_yielding     = false;
  double                        horizon_steps       = 40;
  dynamics::VehicleStateDynamic start_state;
  dynamics::Trajectory          guess_trajectory;
  std::shared_ptr<mas::OCP>     problem;

  enum class MergeState
  {
    NONE,
    EGO_GO,
    EGO_YIELD
  };

  MergeState merge_state_        = MergeState::NONE;
  double     merge_release_time_ = 0.0;

  struct SolverParams
  {
    double max_iterations = 1000;
    double tolerance      = 1e-3;
    double max_ms         = 50;
    double debug          = 1.0;
  } solver_params;

  struct PlannerCostWeights
  {
    double lane_error     = 5.0;
    double long_error     = 0.1;
    double speed_error    = 3.0;
    double heading_error  = 8.0;
    double steering_angle = 1.5;
    double acceleration   = 0.1;
  } weights;

  enum class ManeuverState
  {
    LaneKeep,
    Overtake,
    ReturnToLane
  };

  MotionPlannerConfig config;

  mutable ManeuverState   maneuver_state_ = ManeuverState::LaneKeep;
  mutable double          return_start_s_ = 0.0;
  static constexpr double RETURN_LENGTH   = 10.0; // meters

private:

  double ds_                   = 0.3; // longitudinal step [m]
  double dd_                   = 0.2; // lateral step [m]
  double horizon_length_       = 80;  // meters
  double ego_interaction_speed = 3.0;
  bool   overtaking_allowed    = true;
  int    counter               = 0;

  dynamics::PhysicalVehicleParameters vehicle_params;
  dynamics::ComfortSettings           comfort_settings;
  dynamics::Trajectory                reference_trajectory;
  dynamics::Trajectory                previous_trajectory;
};

} // namespace planner
} // namespace adore
