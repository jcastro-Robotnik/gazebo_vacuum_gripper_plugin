#include "vacuum_gripper_plugin/VacuumGripper.hh"

#include <algorithm>
#include <chrono>
#include <limits>
#include <sstream>

#include <gz/common/Console.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Static.hh>
#include <sdf/sdf.hh>

namespace vacuum_gripper_plugin
{

namespace
{
std::vector<std::string> SplitCsv(const std::string &_text)
{
  std::vector<std::string> out;
  std::stringstream ss(_text);
  std::string item;
  while (std::getline(ss, item, ','))
  {
    item.erase(0, item.find_first_not_of(" \t"));
    item.erase(item.find_last_not_of(" \t") + 1);
    if (!item.empty())
      out.push_back(item);
  }
  return out;
}
}  // namespace

VacuumGripper::VacuumGripper() = default;
VacuumGripper::~VacuumGripper() = default;

// ---------------------------------------------------------------------------
void VacuumGripper::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &)
{
  std::cerr << "[VacuumGripper][DBG] Configure() entity=" << _entity << "\n";

  this->modelEntity = _entity;
  this->model = gz::sim::Model(_entity);

  if (!this->model.Valid(_ecm))
  {
    gzerr << "[VacuumGripper] El plugin debe cargarse sobre un modelo.\n";
    return;
  }

  if (_sdf && _sdf->HasElement("suction_link"))
    this->suctionLinkName = _sdf->Get<std::string>("suction_link");
  if (_sdf && _sdf->HasElement("contact_topic"))
    this->contactTopic = _sdf->Get<std::string>("contact_topic");
  if (_sdf && _sdf->HasElement("cmd_topic"))
    this->cmdTopic = _sdf->Get<std::string>("cmd_topic");
  if (_sdf && _sdf->HasElement("max_distance"))
    this->maxDistance = _sdf->Get<double>("max_distance");
  if (_sdf && _sdf->HasElement("allowed_prefixes"))
    this->allowedPrefixes = SplitCsv(_sdf->Get<std::string>("allowed_prefixes"));

  std::cerr << "[VacuumGripper][DBG] SDF: suction_link=" << this->suctionLinkName
            << " cmd_topic=" << this->cmdTopic
            << " contact_topic=" << this->contactTopic
            << " max_distance=" << this->maxDistance << "\n";

  this->suctionLinkEntity = this->model.LinkByName(_ecm, this->suctionLinkName);
  if (this->suctionLinkEntity == gz::sim::kNullEntity)
  {
    gzerr << "[VacuumGripper] No se encontro suction_link=[" << this->suctionLinkName << "]\n";
    return;
  }
  std::cerr << "[VacuumGripper][DBG] suctionLinkEntity=" << this->suctionLinkEntity << "\n";

  if (!this->node.Subscribe(this->cmdTopic, &VacuumGripper::OnVacuumCmd, this))
  {
    gzerr << "[VacuumGripper] Fallo subscribe cmd_topic=[" << this->cmdTopic << "]\n";
    return;
  }
  if (!this->node.Subscribe(this->contactTopic, &VacuumGripper::OnContacts, this))
  {
    gzerr << "[VacuumGripper] Fallo subscribe contact_topic=[" << this->contactTopic << "]\n";
    return;
  }

  gzmsg << "[VacuumGripper] Cargado correctamente\n"
        << "  suction_link    : " << this->suctionLinkName << "\n"
        << "  contact_topic   : " << this->contactTopic << "\n"
        << "  cmd_topic       : " << this->cmdTopic << "\n"
        << "  max_distance    : " << this->maxDistance << "\n";

  if (!this->allowedPrefixes.empty())
  {
    gzmsg << "  allowed_prefixes: ";
    for (const auto &p : this->allowedPrefixes) gzmsg << p << " ";
    gzmsg << "\n";
  }
  else { gzmsg << "  allowed_prefixes: <cualquiera>\n"; }

  this->configured = true;
  std::cerr << "[VacuumGripper][DBG] Configure() OK\n";
}

// ---------------------------------------------------------------------------
void VacuumGripper::OnVacuumCmd(const gz::msgs::Boolean &_msg)
{
  std::cerr << "[VacuumGripper][DBG] OnVacuumCmd() data=" << _msg.data() << "\n";
  std::lock_guard<std::mutex> lock(this->mutex);
  this->vacuumOn = _msg.data();
  gzmsg << "[VacuumGripper] Comando recibido: vacuum_on=" << this->vacuumOn << "\n";
}

// ---------------------------------------------------------------------------
void VacuumGripper::OnContacts(const gz::msgs::Contacts &_msg)
{
  std::cerr << "[VacuumGripper][DBG] OnContacts() contact_size=" << _msg.contact_size() << "\n";
  std::lock_guard<std::mutex> lock(this->mutex);
  this->contactCandidates.clear();

  for (int i = 0; i < _msg.contact_size(); ++i)
  {
    const auto &contact = _msg.contact(i);

    gz::math::Vector3d contactPt{0.0, 0.0, 0.0};
    if (contact.position_size() > 0)
    {
      const auto &p = contact.position(0);
      contactPt.Set(p.x(), p.y(), p.z());
    }

    std::cerr << "[VacuumGripper][DBG] contacto[" << i << "]"
              << " col1.id=" << (contact.has_collision1() ? std::to_string(contact.collision1().id()) : "N/A")
              << " col2.id=" << (contact.has_collision2() ? std::to_string(contact.collision2().id()) : "N/A")
              << " pt=(" << contactPt.X() << "," << contactPt.Y() << "," << contactPt.Z() << ")\n";

    if (contact.has_collision1())
    {
      ContactCandidate c;
      c.collisionEntity   = static_cast<gz::sim::Entity>(contact.collision1().id());
      c.contactPointWorld = contactPt;
      this->contactCandidates.push_back(c);
    }
    if (contact.has_collision2())
    {
      ContactCandidate c;
      c.collisionEntity   = static_cast<gz::sim::Entity>(contact.collision2().id());
      c.contactPointWorld = contactPt;
      this->contactCandidates.push_back(c);
    }
  }
  std::cerr << "[VacuumGripper][DBG] contactCandidates.size()=" << this->contactCandidates.size() << "\n";
}

// ---------------------------------------------------------------------------
bool VacuumGripper::IsAllowedModel(const std::string &_name) const
{
  if (this->allowedPrefixes.empty()) return true;
  for (const auto &prefix : this->allowedPrefixes)
    if (_name.rfind(prefix, 0) == 0) return true;
  return false;
}

// ---------------------------------------------------------------------------
// Release: limpia estado y elimina los velocity commands del modelo soltado
// usando typeId (forma mas robusta en gz-sim8).
// ---------------------------------------------------------------------------
void VacuumGripper::Release(gz::sim::EntityComponentManager *_ecm)
{
  std::cerr << "[VacuumGripper][DBG] Release() modelo=[" << this->heldModelName
            << "] entity=" << this->heldModelEntity << "\n";

  if (_ecm && this->heldModelEntity != gz::sim::kNullEntity)
  {
    // Usar typeId directamente: mas robusto que la version template en gz-sim8.
    const bool linRemoved = _ecm->RemoveComponent(
        this->heldModelEntity,
        gz::sim::components::LinearVelocityCmd::typeId);
    const bool angRemoved = _ecm->RemoveComponent(
        this->heldModelEntity,
        gz::sim::components::AngularVelocityCmd::typeId);

    std::cerr << "[VacuumGripper][DBG] Release() LinearVelocityCmd removed=" << linRemoved
              << " AngularVelocityCmd removed=" << angRemoved << "\n";
  }

  this->holding         = false;
  this->needsOffsetInit = false;
  this->heldModelEntity = gz::sim::kNullEntity;
  this->heldModelName.clear();
  this->heldOffsetFromSuction = gz::math::Pose3d::Zero;
  // Resetear integradores para el proximo agarre.
  this->integralLin = gz::math::Vector3d::Zero;
  this->integralAng = gz::math::Vector3d::Zero;
  std::cerr << "[VacuumGripper][DBG] Release() integradores reseteados.\n";
}

// ---------------------------------------------------------------------------
// PostUpdate: detecta candidato de agarre por distancia al punto real de
// contacto. Solo anota la entidad; el offset y el control de velocidad
// empiezan en PreUpdate del tick siguiente.
// ---------------------------------------------------------------------------
void VacuumGripper::PostUpdate(
  const gz::sim::UpdateInfo &_info,
  const gz::sim::EntityComponentManager &_ecm)
{
  if (!this->configured || _info.paused) return;

  std::lock_guard<std::mutex> lock(this->mutex);

  std::cerr << "[VacuumGripper][DBG] PostUpdate() vacuumOn=" << this->vacuumOn
            << " holding=" << this->holding
            << " candidates=" << this->contactCandidates.size() << "\n";

  if (!this->vacuumOn || this->holding) return;
  if (this->contactCandidates.empty()) return;

  const auto suctionPoseOpt = gz::sim::Link(this->suctionLinkEntity).WorldPose(_ecm);
  if (!suctionPoseOpt.has_value())
  {
    gzwarn << "[VacuumGripper] No se pudo obtener WorldPose del suction_link.\n";
    return;
  }
  const auto suctionPose = *suctionPoseOpt;

  double bestDist           = this->maxDistance;
  gz::sim::Entity bestModel = gz::sim::kNullEntity;
  std::string bestName;

  for (const auto &candidate : this->contactCandidates)
  {
    if (candidate.collisionEntity == gz::sim::kNullEntity) continue;

    const gz::sim::Entity topModel =
        gz::sim::topLevelModel(candidate.collisionEntity, _ecm);
    if (topModel == gz::sim::kNullEntity) continue;
    if (topModel == this->modelEntity) continue;

    const auto *nameComp = _ecm.Component<gz::sim::components::Name>(topModel);
    const std::string modelName = nameComp ? nameComp->Data() : "";
    if (!this->IsAllowedModel(modelName)) continue;

    const auto *staticComp = _ecm.Component<gz::sim::components::Static>(topModel);
    if (staticComp && staticComp->Data()) continue;

    const double dist = candidate.contactPointWorld.Distance(suctionPose.Pos());

    std::cerr << "[VacuumGripper][DBG] PostUpdate() candidato=[" << modelName
              << "] dist_contact=" << dist << " max=" << this->maxDistance << "\n";

    gzmsg << "[VacuumGripper] Candidato modelo=[" << modelName
          << "] entity=" << topModel
          << " dist_contact=" << dist << " max=" << this->maxDistance << "\n";

    if (dist <= bestDist)
    {
      bestDist  = dist;
      bestModel = topModel;
      bestName  = modelName;
    }
  }

  if (bestModel == gz::sim::kNullEntity)
  {
    std::cerr << "[VacuumGripper][DBG] PostUpdate() sin candidato valido.\n";
    return;
  }

  this->heldModelEntity = bestModel;
  this->heldModelName   = bestName;
  this->holding         = true;
  this->needsOffsetInit = true;  // offset se calcula en PreUpdate (misma fase que el control)

  gzmsg << "[VacuumGripper] Agarre detectado modelo=[" << bestName
        << "] dist_contact=" << bestDist << " m\n";
  std::cerr << "[VacuumGripper][DBG] PostUpdate() holding=true needsOffsetInit=true modelo=["
            << bestName << "] entity=" << bestModel << "\n";
}

// ---------------------------------------------------------------------------
// PreUpdate: controla la posicion del objeto agarrado mediante un controlador
// proporcional sobre LinearVelocityCmd / AngularVelocityCmd.
//
// Por que velocidades en vez de escribir en Pose:
//   El motor de fisica (Bullet/DART) mantiene su propio estado interno de
//   posiciones y velocidades. El componente Pose del ECM es SALIDA del motor,
//   no entrada: escribir en el hace que el SceneBroadcaster lo propague a la
//   GUI, pero el motor lo sobreescribe en el siguiente paso. LinearVelocityCmd
//   y AngularVelocityCmd SI son leidos por el sistema Physics y aplicados al
//   cuerpo rigido interno, por lo que son el mecanismo correcto.
//
// Al soltar, se eliminan los componentes con RemoveComponent(typeId) para
// que el motor de fisica recupere el control completo del objeto.
// ---------------------------------------------------------------------------
void VacuumGripper::PreUpdate(
  const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &_ecm)
{
  if (!this->configured || _info.paused) return;

  std::lock_guard<std::mutex> lock(this->mutex);

  std::cerr << "[VacuumGripper][DBG] PreUpdate() vacuumOn=" << this->vacuumOn
            << " holding=" << this->holding
            << " needsOffsetInit=" << this->needsOffsetInit
            << " heldEntity=" << this->heldModelEntity << "\n";

  if (!this->vacuumOn)
  {
    if (this->holding)
    {
      gzmsg << "[VacuumGripper] Soltando modelo [" << this->heldModelName << "]\n";
      std::cerr << "[VacuumGripper][DBG] PreUpdate() vacuumOn=0 -> Release()\n";
      this->Release(&_ecm);
    }
    return;
  }

  if (!this->holding) return;

  if (this->heldModelEntity == gz::sim::kNullEntity)
  {
    gzwarn << "[VacuumGripper] heldModelEntity invalido, liberando.\n";
    this->Release(&_ecm);
    return;
  }

  const auto suctionPoseOpt = gz::sim::Link(this->suctionLinkEntity).WorldPose(_ecm);
  if (!suctionPoseOpt.has_value()) return;
  const auto suctionPose = *suctionPoseOpt;

  // Primer tick tras agarre: calcular offset en la misma fase que el control.
  if (this->needsOffsetInit)
  {
    const auto modelPose = gz::sim::worldPose(this->heldModelEntity, _ecm);
    this->heldOffsetFromSuction = suctionPose.Inverse() * modelPose;
    this->needsOffsetInit = false;
    std::cerr << "[VacuumGripper][DBG] PreUpdate() offset inicializado. modelPose=("
              << modelPose.Pos().X() << "," << modelPose.Pos().Y() << "," << modelPose.Pos().Z() << ")\n";
    gzmsg << "[VacuumGripper] Agarrado modelo [" << this->heldModelName << "] offset fijado.\n";
  }

  const auto targetPose  = suctionPose * this->heldOffsetFromSuction;
  const auto currentPose = gz::sim::worldPose(this->heldModelEntity, _ecm);

  // Controlador PI:
  //   kP agresivo (1.5/dt) para seguimiento rapido.
  //   kI acumula el error residual para eliminar offset estatico cuando el
  //       objeto tiene peso y el P solo no alcanza.
  //   Anti-windup: el integrador se satura a kIMaxLin / kIMaxAng para evitar
  //       sobrepaso cuando el error es grande al inicio del agarre.
  const double dt = std::max(
      std::chrono::duration<double>(_info.dt).count(), 1e-6);
  const double kP  = 1.5 / dt;   // mas agresivo que antes (era 0.5/dt)
  const double kI  = 0.8 / dt;   // termino integral

  const auto posError = targetPose.Pos() - currentPose.Pos();

  // Acumular integral lineal con saturacion (anti-windup).
  this->integralLin += posError * (kI * dt);
  const double iLinLen = this->integralLin.Length();
  if (iLinLen > this->kIMaxLin)
    this->integralLin = this->integralLin * (this->kIMaxLin / iLinLen);

  const gz::math::Vector3d linVelCmd = posError * kP + this->integralLin;

  // Error de orientacion y termino integral angular.
  const auto rotErr = targetPose.Rot() * currentPose.Rot().Inverse();
  gz::math::Vector3d rotAxis{0.0, 0.0, 1.0};
  double rotAngle = 0.0;
  rotErr.AxisAngle(rotAxis, rotAngle);
  const gz::math::Vector3d angError = rotAxis * rotAngle;

  this->integralAng += angError * (kI * dt);
  const double iAngLen = this->integralAng.Length();
  if (iAngLen > this->kIMaxAng)
    this->integralAng = this->integralAng * (this->kIMaxAng / iAngLen);

  const gz::math::Vector3d angVelCmd = angError * kP + this->integralAng;

  std::cerr << "[VacuumGripper][DBG] PreUpdate() targetPos=("
            << targetPose.Pos().X() << "," << targetPose.Pos().Y() << "," << targetPose.Pos().Z() << ")"
            << " posError=(" << posError.X() << "," << posError.Y() << "," << posError.Z() << ")"
            << " linVelCmd=(" << linVelCmd.X() << "," << linVelCmd.Y() << "," << linVelCmd.Z() << ")"
            << " integralLin=(" << this->integralLin.X() << "," << this->integralLin.Y() << "," << this->integralLin.Z() << ")\n";

  // Aplicar LinearVelocityCmd.
  auto *linComp = _ecm.Component<gz::sim::components::LinearVelocityCmd>(
      this->heldModelEntity);
  if (!linComp)
  {
    _ecm.CreateComponent(this->heldModelEntity,
        gz::sim::components::LinearVelocityCmd(linVelCmd));
    std::cerr << "[VacuumGripper][DBG] PreUpdate() LinearVelocityCmd creado.\n";
  }
  else
  {
    linComp->SetData(linVelCmd, [](auto &, const auto &) { return true; });
  }

  // Aplicar AngularVelocityCmd.
  auto *angComp = _ecm.Component<gz::sim::components::AngularVelocityCmd>(
      this->heldModelEntity);
  if (!angComp)
  {
    _ecm.CreateComponent(this->heldModelEntity,
        gz::sim::components::AngularVelocityCmd(angVelCmd));
    std::cerr << "[VacuumGripper][DBG] PreUpdate() AngularVelocityCmd creado.\n";
  }
  else
  {
    angComp->SetData(angVelCmd, [](auto &, const auto &) { return true; });
  }
}

}  // namespace vacuum_gripper_plugin

GZ_ADD_PLUGIN(
  vacuum_gripper_plugin::VacuumGripper,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate,
  gz::sim::ISystemPostUpdate
)
