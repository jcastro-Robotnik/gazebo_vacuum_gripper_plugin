#pragma once

#include <mutex>
#include <string>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/contacts.pb.h>
#include <gz/sim/Entity.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace vacuum_gripper_plugin
{

/// Informacion de un candidato a ser agarrado, extraida del mensaje de contacto
struct ContactCandidate
{
  gz::sim::Entity    collisionEntity{gz::sim::kNullEntity};
  gz::sim::Entity    modelEntity{gz::sim::kNullEntity};
  std::string        modelName;
  gz::math::Vector3d contactPointWorld;   // punto real de colision en coords mundo
  double             distanceToSuction{std::numeric_limits<double>::max()};
};

class VacuumGripper:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemPostUpdate
{
public:
  VacuumGripper();
  ~VacuumGripper() override;

  void Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_eventMgr) override;

  void PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) override;

  void PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm) override;

private:
  void OnVacuumCmd(const gz::msgs::Boolean &_msg);
  void OnContacts(const gz::msgs::Contacts &_msg);

  bool IsAllowedModel(const std::string &_name) const;

  // Suelta el objeto; si se pasa _ecm limpia los componentes de velocidad
  void Release(gz::sim::EntityComponentManager *_ecm = nullptr);

private:
  gz::sim::Model  model{gz::sim::kNullEntity};
  gz::sim::Entity modelEntity{gz::sim::kNullEntity};
  gz::sim::Entity suctionLinkEntity{gz::sim::kNullEntity};

  std::string suctionLinkName{"vacuum_link_suction_gripper"};
  std::string contactTopic{"/suction_contact"};
  std::string cmdTopic{"/vacuum_on"};

  double maxDistance{0.25};
  std::vector<std::string> allowedPrefixes;

  gz::transport::Node node;
  std::mutex mutex;

  bool configured{false};
  bool vacuumOn{false};
  bool holding{false};

  // true en el primer PreUpdate tras detectar agarre:
  // permite calcular el offset en la misma fase en que se aplica (evita teleport)
  bool needsOffsetInit{false};

  gz::sim::Entity  heldModelEntity{gz::sim::kNullEntity};
  std::string      heldModelName;
  gz::math::Pose3d heldOffsetFromSuction;

  // Controlador PI
  gz::math::Vector3d integralLin{gz::math::Vector3d::Zero};
  gz::math::Vector3d integralAng{gz::math::Vector3d::Zero};
  static constexpr double kIMaxLin{2.0};   // umbral anti-windup lineal (m/s)
  static constexpr double kIMaxAng{1.5};   // umbral anti-windup angular (rad/s)

  // Candidatos con punto de contacto real, actualizados en cada tick del sensor
  std::vector<ContactCandidate> contactCandidates;
};

}  // namespace vacuum_gripper_plugin
