// Copyright 2026 AVATIC contributors.
//
// Gazebo Harmonic world system: the AVATIC balloon arena.
//
// Configure (world plugin):
//   * reads the arena YAML (<config_file>) and the balloon surface profile
//     (<profile_file>, generated with the mesh by tools/prepare_balloon_mesh.py)
//   * spawns one static, visual-only balloon model per entry, scaled to
//     balloon.diameter_m and coloured per its colour
// PreUpdate, every physics step:
//   * starts the run clock (simulation start, or the first ARMED telemetry
//     frame, per clock_start)
//   * pluto_x::ArenaScoring::Update with the vehicle link position: a
//     balloon touched by the vehicle's contact sphere is popped (its model
//     removed) and scores its colour's points
//   * at time_limit_s of run time the run ends: no further scoring, the
//     result is published (and written to <result_file> if given), and the
//     world is paused (pause_on_time_limit)
//
// Topics (gz-transport; bridged to ROS by pluto_x_bringup arena bridge):
//   /arena/score           gz.msgs.Int32     total points (on change + 10 Hz)
//   /arena/time_remaining  gz.msgs.Double    s of run time left (10 Hz)
//   /arena/events          gz.msgs.StringMsg one line per event (start, pop, end)
//   /arena/result          gz.msgs.StringMsg final summary (YAML), at the end
// Balloon positions are NOT published (they are what the participants must
// find).
//
// SDF parameters:
//   <config_file>   (required) arena YAML
//   <profile_file>  (required) balloon profile YAML
//   <mesh_uri>      (default model://balloon/meshes/balloon_unit.stl)
//   <telemetry_topic> (default /pluto/fc_telemetry), for clock_start: armed
//   <result_file>   (optional) path to write the final result YAML

#ifndef PLUTO_X_GAZEBO_ARENA_SYSTEM_HPP_
#define PLUTO_X_GAZEBO_ARENA_SYSTEM_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <gz/msgs/double_v.pb.h>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

#include "pluto_x/arena/arena_scoring.hpp"

namespace pluto_x_gazebo {

class ArenaSystem : public gz::sim::System,
                    public gz::sim::ISystemConfigure,
                    public gz::sim::ISystemPreUpdate {
 public:
  void Configure(const gz::sim::Entity& entity,
                 const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm,
                 gz::sim::EventManager& event_manager) override;

  void PreUpdate(const gz::sim::UpdateInfo& info,
                 gz::sim::EntityComponentManager& ecm) override;

 private:
  bool LoadConfig(const std::string& config_path, const std::string& profile_path);
  void SpawnBalloons(gz::sim::EntityComponentManager& ecm);
  gz::sim::Entity ResolveVehicleLink(const gz::sim::EntityComponentManager& ecm);
  void OnTelemetry(const gz::msgs::Double_V& message);
  void Event(double time_s, const std::string& text);
  void Finish(double time_s);

  struct ColorSpec {
    int points{0};
    std::array<double, 4> rgba{};
  };

  bool enabled_{false};
  gz::sim::Entity world_{gz::sim::kNullEntity};
  std::string world_name_;
  gz::sim::EventManager* event_manager_{nullptr};
  std::string mesh_uri_{"model://balloon/meshes/balloon_unit.stl"};
  std::string result_file_;

  // configuration
  double time_limit_s_{15.0};
  bool clock_on_arm_{false};
  bool pause_on_time_limit_{true};
  std::string vehicle_model_{"pluto_x"};
  std::string vehicle_link_{"base_link"};
  double diameter_m_{0.30};
  std::vector<std::pair<std::string, ColorSpec>> colors_;
  std::unique_ptr<pluto_x::ArenaScoring> scoring_;

  std::vector<gz::sim::Entity> balloon_entities_;
  gz::sim::Entity vehicle_link_entity_{gz::sim::kNullEntity};
  bool reported_missing_vehicle_{false};
  std::atomic<bool> armed_seen_{false};
  bool finished_reported_{false};
  std::int64_t next_publish_ns_{0};
  int last_published_score_{-1};

  gz::transport::Node node_;
  gz::transport::Node::Publisher score_pub_;
  gz::transport::Node::Publisher remaining_pub_;
  gz::transport::Node::Publisher events_pub_;
  gz::transport::Node::Publisher result_pub_;
};

}  // namespace pluto_x_gazebo

#endif  // PLUTO_X_GAZEBO_ARENA_SYSTEM_HPP_
