// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <gz/math/Vector3.hh>
#include <gz/msgs/entity_wrench.pb.h>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/transport/Node.hh>

namespace drone_motor_system
{

/// Hold exactly the latest commanded world-frame wrench and apply it during
/// every Gazebo physics PreUpdate. Gazebo's stock instantaneous wrench topic
/// consumes a message for one update only, while its persistent topic appends
/// every message. Neither behavior is suitable for a continuously changing
/// motor wrench delivered across a ROS / Gazebo bridge.
class LatestWrenchSystem final:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
  public: void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &) override
  {
    gz::sim::World world(_entity);
    if (!world.Valid(_ecm))
    {
      gzerr << "LatestWrenchSystem must be attached to a world." << std::endl;
      return;
    }

    this->topic = "/world/" + world.Name(_ecm).value() + "/wrench/latest";
    if (_sdf->HasElement("topic"))
      this->topic = _sdf->Get<std::string>("topic");

    this->timeout = std::chrono::duration<double>(
      _sdf->Get<double>("wall_timeout_s", 1.0).first);
    this->subscriber = this->node.CreateSubscriber(
      this->topic, &LatestWrenchSystem::OnWrench, this);

    gzmsg << "Holding latest wrench from [" << this->topic
          << "] for every physics step; wall timeout ["
          << this->timeout.count() << " s]." << std::endl;
  }

  public: void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;

    std::optional<gz::msgs::EntityWrench> command;
    std::chrono::steady_clock::time_point received;
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      command = this->latest;
      received = this->lastReceived;
    }
    if (!command.has_value())
      return;
    if (this->timeout.count() > 0.0 &&
        std::chrono::steady_clock::now() - received > this->timeout)
      return;

    const auto entity = gz::sim::entityFromMsg(_ecm, command->entity());
    if (entity == gz::sim::kNullEntity)
      return;

    gz::sim::Link link(entity);
    if (!link.Valid(_ecm))
    {
      const gz::sim::Model model(entity);
      if (!model.Valid(_ecm))
        return;
      link = gz::sim::Link(model.CanonicalLink(_ecm));
    }
    if (!link.Valid(_ecm))
      return;

    gz::math::Vector3d force = gz::math::Vector3d::Zero;
    gz::math::Vector3d torque = gz::math::Vector3d::Zero;
    gz::math::Vector3d offset = gz::math::Vector3d::Zero;
    if (command->wrench().has_force())
      force = gz::msgs::Convert(command->wrench().force());
    if (command->wrench().has_torque())
      torque = gz::msgs::Convert(command->wrench().torque());
    if (command->wrench().has_force_offset())
      offset = gz::msgs::Convert(command->wrench().force_offset());
    link.AddWorldWrench(_ecm, force, torque, offset);
  }

  private: void OnWrench(const gz::msgs::EntityWrench &_message)
  {
    if (!_message.has_entity() || !_message.has_wrench())
      return;
    std::lock_guard<std::mutex> lock(this->mutex);
    this->latest = _message;
    this->lastReceived = std::chrono::steady_clock::now();
  }

  private: gz::transport::Node node;
  private: gz::transport::Node::Subscriber subscriber;
  private: std::string topic;
  private: std::chrono::duration<double> timeout{1.0};
  private: std::mutex mutex;
  private: std::optional<gz::msgs::EntityWrench> latest;
  private: std::chrono::steady_clock::time_point lastReceived{};
};

}  // namespace drone_motor_system

GZ_ADD_PLUGIN(
  drone_motor_system::LatestWrenchSystem,
  gz::sim::System,
  drone_motor_system::LatestWrenchSystem::ISystemConfigure,
  drone_motor_system::LatestWrenchSystem::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  drone_motor_system::LatestWrenchSystem,
  "drone_motor_system::LatestWrenchSystem")
