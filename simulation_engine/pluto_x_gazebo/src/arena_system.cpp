// Copyright 2026 AVATIC contributors.

#include "pluto_x_gazebo/arena_system.hpp"

#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>

#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include <gz/common/Console.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/int32.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <sdf/Root.hh>

#include "pluto_x/control/telemetry_wire.hpp"

namespace pluto_x_gazebo {
namespace {

constexpr std::int64_t kPublishPeriodNs = 100000000;  // 10 Hz

std::int64_t ToNanoseconds(const std::chrono::steady_clock::duration& d) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(d).count();
}

double Seconds(std::int64_t ns) { return static_cast<double>(ns) * 1e-9; }

std::string BalloonSdf(const std::string& name, const pluto_x::Vector3& p,
                       double diameter_m, const std::array<double, 4>& rgba,
                       const std::string& mesh_uri) {
  std::ostringstream c;
  c << rgba[0] << ' ' << rgba[1] << ' ' << rgba[2] << ' ' << rgba[3];
  std::ostringstream s;
  s << std::setprecision(9)
    << "<?xml version='1.0'?><sdf version='1.10'><model name='" << name << "'>"
    << "<static>true</static><pose>" << p.x() << ' ' << p.y() << ' ' << p.z()
    << " 0 0 0</pose><link name='link'><visual name='visual'><geometry><mesh>"
    << "<uri>" << mesh_uri << "</uri><scale>" << diameter_m << ' ' << diameter_m
    << ' ' << diameter_m << "</scale></mesh></geometry><material><ambient>"
    << c.str() << "</ambient><diffuse>" << c.str()
    << "</diffuse><specular>0.3 0.3 0.3 1</specular>"
    // latex: dielectric, fairly glossy (ogre2 renders <pbr>; the classic
    // terms are kept for other engines)
    << "<pbr><metal><metalness>0.0</metalness><roughness>0.35</roughness>"
    << "</metal></pbr></material></visual>"
    << "</link></model></sdf>";
  return s.str();
}

}  // namespace

void ArenaSystem::Configure(const gz::sim::Entity& entity,
                            const std::shared_ptr<const sdf::Element>& sdf,
                            gz::sim::EntityComponentManager& ecm,
                            gz::sim::EventManager& event_manager) {
  const gz::sim::World world(entity);
  if (!world.Valid(ecm)) {
    gzerr << "[arena] ArenaSystem must be attached to a world; disabled.\n";
    return;
  }
  world_ = entity;
  world_name_ = world.Name(ecm).value_or("");
  event_manager_ = &event_manager;
  if (!sdf->HasElement("config_file") || !sdf->HasElement("profile_file")) {
    gzerr << "[arena] <config_file> and <profile_file> are required; disabled.\n";
    return;
  }
  if (sdf->HasElement("mesh_uri")) mesh_uri_ = sdf->Get<std::string>("mesh_uri");
  if (sdf->HasElement("result_file")) {
    result_file_ = sdf->Get<std::string>("result_file");
  }
  const std::string telemetry_topic =
      sdf->HasElement("telemetry_topic") ? sdf->Get<std::string>("telemetry_topic")
                                         : std::string("/pluto/fc_telemetry");
  if (!LoadConfig(sdf->Get<std::string>("config_file"),
                  sdf->Get<std::string>("profile_file"))) {
    gzerr << "[arena] disabled.\n";
    return;
  }
  SpawnBalloons(ecm);

  score_pub_ = node_.Advertise<gz::msgs::Int32>("/arena/score");
  remaining_pub_ = node_.Advertise<gz::msgs::Double>("/arena/time_remaining");
  events_pub_ = node_.Advertise<gz::msgs::StringMsg>("/arena/events");
  result_pub_ = node_.Advertise<gz::msgs::StringMsg>("/arena/result");
  if (clock_on_arm_ &&
      !node_.Subscribe(telemetry_topic, &ArenaSystem::OnTelemetry, this)) {
    gzerr << "[arena] cannot subscribe to " << telemetry_topic << "; disabled.\n";
    return;
  }
  gzmsg << "[arena] " << balloon_entities_.size() << " balloons (max "
        << scoring_->max_points() << " points), diameter " << diameter_m_
        << " m, time limit " << time_limit_s_ << " s from "
        << (clock_on_arm_ ? "arming" : "simulation start") << ", vehicle '"
        << vehicle_model_ << "::" << vehicle_link_ << "'.\n";
  enabled_ = true;
}

bool ArenaSystem::LoadConfig(const std::string& config_path,
                             const std::string& profile_path) {
  try {
    const YAML::Node root = YAML::LoadFile(config_path);
    time_limit_s_ = root["time_limit_s"].as<double>();
    const std::string clock = root["clock_start"].as<std::string>();
    if (clock != "sim_start" && clock != "armed") {
      throw std::runtime_error("clock_start must be sim_start or armed");
    }
    clock_on_arm_ = clock == "armed";
    pause_on_time_limit_ = root["pause_on_time_limit"].as<bool>();
    vehicle_model_ = root["vehicle"]["model"].as<std::string>();
    vehicle_link_ = root["vehicle"]["link"].as<std::string>();
    std::vector<pluto_x::ContactSphere> contact_spheres;
    for (const auto& item : root["vehicle"]["contact_spheres"]) {
      const YAML::Node c = item["position_flu_m"];
      if (!c.IsSequence() || c.size() != 3) {
        throw std::runtime_error("vehicle.contact_spheres.*.position_flu_m needs [x, y, z]");
      }
      contact_spheres.push_back(
          {pluto_x::Vector3(c[0].as<double>(), c[1].as<double>(), c[2].as<double>()),
           item["radius_m"].as<double>()});
    }
    diameter_m_ = root["balloon"]["diameter_m"].as<double>();

    for (const auto& item : root["colors"]) {
      ColorSpec spec;
      spec.points = item.second["points"].as<int>();
      const YAML::Node rgba = item.second["rgba"];
      if (!rgba.IsSequence() || rgba.size() != 4) {
        throw std::runtime_error("colors.*.rgba needs 4 values");
      }
      for (int i = 0; i < 4; ++i) spec.rgba[i] = rgba[i].as<double>();
      colors_.emplace_back(item.first.as<std::string>(), spec);
    }
    std::vector<pluto_x::BalloonSpec> balloons;
    for (const auto& item : root["balloons"]) {
      const std::string color = item["color"].as<std::string>();
      const ColorSpec* spec = nullptr;
      for (const auto& c : colors_) {
        if (c.first == color) spec = &c.second;
      }
      if (spec == nullptr) {
        throw std::runtime_error("balloon colour '" + color + "' not in colors");
      }
      const YAML::Node p = item["position_enu_m"];
      if (!p.IsSequence() || p.size() != 3) {
        throw std::runtime_error("balloons.*.position_enu_m needs [x, y, z]");
      }
      pluto_x::BalloonSpec b;
      b.color = color;
      b.points = spec->points;
      b.centre_enu_m = pluto_x::Vector3(p[0].as<double>(), p[1].as<double>(),
                                        p[2].as<double>());
      b.name = "balloon_" + std::to_string(balloons.size()) + "_" + color;
      balloons.push_back(b);
    }
    if (balloons.empty()) throw std::runtime_error("no balloons configured");

    std::vector<std::array<double, 2>> profile;
    for (const auto& point : YAML::LoadFile(profile_path)["profile_unit_diameter"]) {
      profile.push_back({point[0].as<double>(), point[1].as<double>()});
    }
    scoring_ = std::make_unique<pluto_x::ArenaScoring>(
        balloons, pluto_x::BalloonShape(profile, diameter_m_), contact_spheres,
        time_limit_s_);
  } catch (const std::exception& error) {
    gzerr << "[arena] invalid arena configuration (" << config_path << ", "
          << profile_path << "): " << error.what() << "\n";
    return false;
  }
  return true;
}

void ArenaSystem::SpawnBalloons(gz::sim::EntityComponentManager& ecm) {
  gz::sim::SdfEntityCreator creator(ecm, *event_manager_);
  for (const pluto_x::BalloonSpec& b : scoring_->balloons()) {
    std::array<double, 4> rgba{};
    for (const auto& c : colors_) {
      if (c.first == b.color) rgba = c.second.rgba;
    }
    sdf::Root root;
    const sdf::Errors errors =
        root.LoadSdfString(BalloonSdf(b.name, b.centre_enu_m, diameter_m_, rgba, mesh_uri_));
    if (!errors.empty() || root.Model() == nullptr) {
      gzerr << "[arena] cannot build balloon '" << b.name << "'.\n";
      balloon_entities_.push_back(gz::sim::kNullEntity);
      continue;
    }
    const gz::sim::Entity model = creator.CreateEntities(root.Model());
    creator.SetParent(model, world_);
    balloon_entities_.push_back(model);
  }
}

gz::sim::Entity ArenaSystem::ResolveVehicleLink(
    const gz::sim::EntityComponentManager& ecm) {
  if (vehicle_link_entity_ != gz::sim::kNullEntity) return vehicle_link_entity_;
  const gz::sim::Entity model_entity = ecm.EntityByComponents(
      gz::sim::components::Name(vehicle_model_), gz::sim::components::Model());
  if (model_entity == gz::sim::kNullEntity) return gz::sim::kNullEntity;
  vehicle_link_entity_ = gz::sim::Model(model_entity).LinkByName(ecm, vehicle_link_);
  return vehicle_link_entity_;
}

void ArenaSystem::OnTelemetry(const gz::msgs::Double_V& message) {
  namespace wire = pluto_x::telemetry_wire;
  if (message.data_size() == static_cast<int>(wire::kCount) &&
      message.data(wire::kArmed) > 0.5) {
    armed_seen_ = true;
  }
}

void ArenaSystem::Event(double time_s, const std::string& text) {
  std::ostringstream line;
  line << std::fixed << std::setprecision(3) << "t=" << time_s << " s: " << text;
  gzmsg << "[arena] " << line.str() << "\n";
  gz::msgs::StringMsg msg;
  msg.set_data(line.str());
  events_pub_.Publish(msg);
}

void ArenaSystem::Finish(double time_s) {
  std::ostringstream result;
  result << std::fixed << std::setprecision(3) << "time_limit_s: " << time_limit_s_
         << "\nclock_start: " << (clock_on_arm_ ? "armed" : "sim_start")
         << "\nended_at_sim_time_s: " << time_s
         << "\nscore: " << scoring_->total_points()
         << "\nmax_score: " << scoring_->max_points()
         << "\npopped: " << scoring_->popped_count() << "\nballoons:\n";
  for (std::size_t i = 0; i < scoring_->balloons().size(); ++i) {
    const pluto_x::BalloonSpec& b = scoring_->balloons()[i];
    result << "  - {name: " << b.name << ", color: " << b.color
           << ", points: " << b.points
           << ", popped: " << (scoring_->popped(i) ? "true" : "false") << "}\n";
  }
  Event(time_s, "TIME UP - score " + std::to_string(scoring_->total_points()) +
                    " / " + std::to_string(scoring_->max_points()) + " (" +
                    std::to_string(scoring_->popped_count()) + " balloons)");
  gz::msgs::StringMsg msg;
  msg.set_data(result.str());
  result_pub_.Publish(msg);
  if (!result_file_.empty()) {
    std::ofstream(result_file_) << result.str();
  }
  if (pause_on_time_limit_ && !world_name_.empty()) {
    gz::msgs::WorldControl request;
    request.set_pause(true);
    std::function<void(const gz::msgs::Boolean&, const bool)> done =
        [](const gz::msgs::Boolean&, const bool ok) {
          if (!ok) gzerr << "[arena] pause request failed.\n";
        };
    node_.Request("/world/" + world_name_ + "/control", request, done);
  }
}

void ArenaSystem::PreUpdate(const gz::sim::UpdateInfo& info,
                            gz::sim::EntityComponentManager& ecm) {
  if (!enabled_ || info.paused || finished_reported_) return;
  const std::int64_t now_ns = ToNanoseconds(info.simTime);
  const double now_s = Seconds(now_ns);

  if (!scoring_->clock_started() && (!clock_on_arm_ || armed_seen_)) {
    scoring_->StartClock(now_s);
    std::ostringstream text;
    text << "run clock started (limit " << time_limit_s_ << " s)";
    Event(now_s, text.str());
  }
  const gz::sim::Entity link = ResolveVehicleLink(ecm);
  if (link == gz::sim::kNullEntity) {
    if (!reported_missing_vehicle_ && now_s > 1.0) {
      gzwarn << "[arena] vehicle '" << vehicle_model_ << "::" << vehicle_link_
             << "' not found yet; no scoring.\n";
      reported_missing_vehicle_ = true;
    }
  } else {
    const gz::math::Pose3d pose = gz::sim::worldPose(link, ecm);
    const gz::math::Quaterniond& q = pose.Rot();
    const pluto_x::Matrix3 world_from_body =
        Eigen::Quaterniond(q.W(), q.X(), q.Y(), q.Z()).toRotationMatrix();
    const gz::math::Vector3d p = pose.Pos();
    for (const pluto_x::PopEvent& e : scoring_->Update(
             now_s, pluto_x::Vector3(p.X(), p.Y(), p.Z()), world_from_body)) {
      const pluto_x::BalloonSpec& b = scoring_->balloons()[e.index];
      if (balloon_entities_[e.index] != gz::sim::kNullEntity) {
        gz::sim::SdfEntityCreator(ecm, *event_manager_)
            .RequestRemoveEntity(balloon_entities_[e.index], true);
        balloon_entities_[e.index] = gz::sim::kNullEntity;
      }
      std::ostringstream text;
      text << std::fixed << std::setprecision(2) << "POP " << b.color << " "
           << std::showpos << e.points << std::noshowpos << " (" << b.name << ", run time " << e.run_time_s
           << " s) - total " << e.total_points;
      Event(now_s, text.str());
    }
  }
  if (scoring_->finished()) {
    finished_reported_ = true;
    Finish(now_s);
  }
  if (now_ns >= next_publish_ns_ || scoring_->total_points() != last_published_score_ ||
      finished_reported_) {
    next_publish_ns_ = now_ns + kPublishPeriodNs;
    last_published_score_ = scoring_->total_points();
    gz::msgs::Int32 score;
    score.set_data(scoring_->total_points());
    score_pub_.Publish(score);
    gz::msgs::Double remaining;
    remaining.set_data(scoring_->remaining_s(now_s));
    remaining_pub_.Publish(remaining);
  }
}

}  // namespace pluto_x_gazebo

GZ_ADD_PLUGIN(pluto_x_gazebo::ArenaSystem, gz::sim::System,
              gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
