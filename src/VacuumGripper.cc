#include "vacuum_gripper_plugin/VacuumGripper.hh"

#include <algorithm>
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
#include <gz/sim/components/PoseCmd.hh>
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
VacuumGripper::~VacuumGripper()
{
  this->rosContactPub.reset();
  this->rosNode.reset();
  if (this->rosContext) {
    this->rosContext->shutdown("VacuumGripper plugin unloaded");
    this->rosContext.reset();
  }
}

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

  if (_sdf && _sdf->HasElement("ros_contact_topic"))
    this->rosContactTopic = _sdf->Get<std::string>("ros_contact_topic");

  if (_sdf && _sdf->HasElement("max_distance"))
    this->maxDistance = _sdf->Get<double>("max_distance");

  if (_sdf && _sdf->HasElement("allowed_prefixes"))
    this->allowedPrefixes = SplitCsv(_sdf->Get<std::string>("allowed_prefixes"));

  std::cerr << "[VacuumGripper][DBG] SDF:"
            << " suction_link=" << this->suctionLinkName
            << " cmd_topic=" << this->cmdTopic
            << " contact_topic=" << this->contactTopic
            << " ros_contact_topic=" << this->rosContactTopic
            << " max_distance=" << this->maxDistance
            << "\n";

  this->suctionLinkEntity = this->model.LinkByName(_ecm, this->suctionLinkName);

  if (this->suctionLinkEntity == gz::sim::kNullEntity)
  {
    gzerr << "[VacuumGripper] No se encontro suction_link=["
          << this->suctionLinkName << "]\n";
    return;
  }

  std::cerr << "[VacuumGripper][DBG] suctionLinkEntity="
            << this->suctionLinkEntity << "\n";

  if (!this->node.Subscribe(this->cmdTopic, &VacuumGripper::OnVacuumCmd, this))
  {
    gzerr << "[VacuumGripper] Fallo subscribe cmd_topic=["
          << this->cmdTopic << "]\n";
    return;
  }

  if (!this->node.Subscribe(this->contactTopic, &VacuumGripper::OnContacts, this))
  {
    gzerr << "[VacuumGripper] Fallo subscribe contact_topic=["
          << this->contactTopic << "]\n";
    return;
  }

  try
  {
    this->rosContext = std::make_shared<rclcpp::Context>();
    this->rosContext->init(0, nullptr);
    rclcpp::NodeOptions options;
    options.context(this->rosContext);
    this->rosNode = std::make_shared<rclcpp::Node>("vacuum_gripper_contact_publisher", options);
    this->rosContactPub =
      this->rosNode->create_publisher<std_msgs::msg::Bool>(this->rosContactTopic, 10);
    this->PublishRosContact(false);
  }
  catch (const std::exception &e)
  {
    gzerr << "[VacuumGripper] No se pudo crear publicador ROS de contacto: "
          << e.what() << "\n";
    return;
  }

  gzmsg << "[VacuumGripper] Cargado correctamente\n"
        << "  suction_link    : " << this->suctionLinkName << "\n"
        << "  contact_topic   : " << this->contactTopic << "\n"
        << "  ros_contact     : " << this->rosContactTopic << "\n"
        << "  cmd_topic       : " << this->cmdTopic << "\n"
        << "  max_distance    : " << this->maxDistance << "\n";

  if (!this->allowedPrefixes.empty())
  {
    gzmsg << "  allowed_prefixes: ";
    for (const auto &p : this->allowedPrefixes)
      gzmsg << p << " ";
    gzmsg << "\n";
  }
  else
  {
    gzmsg << "  allowed_prefixes: <cualquiera>\n";
  }

  this->configured = true;
  std::cerr << "[VacuumGripper][DBG] Configure() OK\n";
}

// ---------------------------------------------------------------------------
void VacuumGripper::OnVacuumCmd(const gz::msgs::Boolean &_msg)
{
  std::cerr << "[VacuumGripper][DBG] OnVacuumCmd() data="
            << _msg.data() << "\n";

  std::lock_guard<std::mutex> lock(this->mutex);

  this->vacuumOn = _msg.data();
  if (!this->vacuumOn)
  {
    this->contactCandidates.clear();
  }

  gzmsg << "[VacuumGripper] Comando recibido: vacuum_on="
        << this->vacuumOn << "\n";
}

// ---------------------------------------------------------------------------
void VacuumGripper::OnContacts(const gz::msgs::Contacts &_msg)
{
  std::cerr << "[VacuumGripper][DBG] OnContacts() contact_size="
            << _msg.contact_size() << "\n";

  std::lock_guard<std::mutex> lock(this->mutex);

  this->PublishRosContact(_msg.contact_size() > 0);

  this->contactCandidates.clear();
  if (!this->vacuumOn)
  {
    std::cerr << "[VacuumGripper][DBG] OnContacts() ignorado: vacuumOff\n";
    return;
  }

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
              << " col1.id="
              << (contact.has_collision1()
                    ? std::to_string(contact.collision1().id())
                    : "N/A")
              << " col2.id="
              << (contact.has_collision2()
                    ? std::to_string(contact.collision2().id())
                    : "N/A")
              << " pt=("
              << contactPt.X() << ","
              << contactPt.Y() << ","
              << contactPt.Z() << ")\n";

    if (contact.has_collision1())
    {
      ContactCandidate c;
      c.collisionEntity =
          static_cast<gz::sim::Entity>(contact.collision1().id());
      c.contactPointWorld = contactPt;
      this->contactCandidates.push_back(c);
    }

    if (contact.has_collision2())
    {
      ContactCandidate c;
      c.collisionEntity =
          static_cast<gz::sim::Entity>(contact.collision2().id());
      c.contactPointWorld = contactPt;
      this->contactCandidates.push_back(c);
    }
  }

  std::cerr << "[VacuumGripper][DBG] contactCandidates.size()="
            << this->contactCandidates.size() << "\n";
}

// ---------------------------------------------------------------------------
void VacuumGripper::PublishRosContact(bool _contact)
{
  if (!this->rosContactPub)
    return;

  if (_contact == this->lastPublishedContact)
    return;

  std_msgs::msg::Bool msg;
  msg.data = _contact;
  this->rosContactPub->publish(msg);
  this->lastPublishedContact = _contact;
}

// ---------------------------------------------------------------------------
bool VacuumGripper::IsAllowedModel(const std::string &_name) const
{
  if (this->allowedPrefixes.empty())
    return true;

  for (const auto &prefix : this->allowedPrefixes)
  {
    if (_name.rfind(prefix, 0) == 0)
      return true;
  }

  return false;
}

// ---------------------------------------------------------------------------
// Release:
// Limpia el estado de agarre y elimina los comandos que hacían que el modelo
// agarrado se comportara de forma cinemática. Al quitar WorldPoseCmd y las
// velocidades, Gazebo vuelve a controlar el objeto con física normal.
// ---------------------------------------------------------------------------
void VacuumGripper::Release(gz::sim::EntityComponentManager *_ecm)
{
  std::cerr << "[VacuumGripper][DBG] Release() modelo=["
            << this->heldModelName
            << "] entity=" << this->heldModelEntity << "\n";

  if (_ecm && this->heldModelEntity != gz::sim::kNullEntity)
  {
    const bool linRemoved = _ecm->RemoveComponent(
      this->heldModelEntity,
      gz::sim::components::LinearVelocityCmd::typeId);

    const bool angRemoved = _ecm->RemoveComponent(
      this->heldModelEntity,
      gz::sim::components::AngularVelocityCmd::typeId);

    const bool poseRemoved = _ecm->RemoveComponent(
      this->heldModelEntity,
      gz::sim::components::WorldPoseCmd::typeId);

    std::cerr << "[VacuumGripper][DBG] Release()"
              << " LinearVelocityCmd removed=" << linRemoved
              << " AngularVelocityCmd removed=" << angRemoved
              << " WorldPoseCmd removed=" << poseRemoved
              << "\n";
  }

  this->holding = false;
  this->needsOffsetInit = false;
  this->heldModelEntity = gz::sim::kNullEntity;
  this->heldModelName.clear();
  this->heldOffsetFromSuction = gz::math::Pose3d::Zero;
  this->heldContactPointWorld = gz::math::Vector3d::Zero;
  this->contactCandidates.clear();

  std::cerr << "[VacuumGripper][DBG] Release() comandos cinematicos limpiados.\n";
}

// ---------------------------------------------------------------------------
// PostUpdate:
// Detecta candidatos de agarre a partir de los contactos recibidos.
// La distancia se mide entre el punto de contacto y el suction_link.
//
// Importante:
// Si en el URDF usas un link auxiliar en la punta de la ventosa, por ejemplo
// vacuum_suction_tip_link, entonces suctionPose.Pos() representa la punta real.
// ---------------------------------------------------------------------------
void VacuumGripper::PostUpdate(
  const gz::sim::UpdateInfo &_info,
  const gz::sim::EntityComponentManager &_ecm)
{
  if (!this->configured || _info.paused)
    return;

  std::lock_guard<std::mutex> lock(this->mutex);

  std::cerr << "[VacuumGripper][DBG] PostUpdate()"
            << " vacuumOn=" << this->vacuumOn
            << " holding=" << this->holding
            << " candidates=" << this->contactCandidates.size()
            << "\n";

  if (!this->vacuumOn)
  {
    this->contactCandidates.clear();
    return;
  }

  if (this->holding)
    return;

  if (this->contactCandidates.empty())
    return;

  const auto suctionPoseOpt =
      gz::sim::Link(this->suctionLinkEntity).WorldPose(_ecm);

  if (!suctionPoseOpt.has_value())
  {
    gzwarn << "[VacuumGripper] No se pudo obtener WorldPose del suction_link.\n";
    return;
  }

  const auto suctionPose = *suctionPoseOpt;

  double bestDist = std::numeric_limits<double>::max();
  gz::sim::Entity bestModel = gz::sim::kNullEntity;
  std::string bestName;
  gz::math::Vector3d bestContactPointWorld{0.0, 0.0, 0.0};

  for (const auto &candidate : this->contactCandidates)
  {
    if (candidate.collisionEntity == gz::sim::kNullEntity)
      continue;

    const gz::sim::Entity topModel =
        gz::sim::topLevelModel(candidate.collisionEntity, _ecm);

    if (topModel == gz::sim::kNullEntity)
      continue;

    if (topModel == this->modelEntity)
      continue;

    const auto *nameComp =
        _ecm.Component<gz::sim::components::Name>(topModel);

    const std::string modelName = nameComp ? nameComp->Data() : "";

    if (!this->IsAllowedModel(modelName))
      continue;

    const auto *staticComp =
        _ecm.Component<gz::sim::components::Static>(topModel);

    if (staticComp && staticComp->Data())
      continue;

    const double dist =
        candidate.contactPointWorld.Distance(suctionPose.Pos());

    std::cerr << "[VacuumGripper][DBG] PostUpdate() candidato=["
              << modelName
              << "] dist_contact=" << dist
              << " max=" << this->maxDistance
              << " suctionTip=("
              << suctionPose.Pos().X() << ","
              << suctionPose.Pos().Y() << ","
              << suctionPose.Pos().Z() << ")"
              << " contact=("
              << candidate.contactPointWorld.X() << ","
              << candidate.contactPointWorld.Y() << ","
              << candidate.contactPointWorld.Z() << ")"
              << "\n";

    gzmsg << "[VacuumGripper] Candidato modelo=["
          << modelName
          << "] entity=" << topModel
          << " dist_contact=" << dist
          << " max=" << this->maxDistance
          << "\n";

    if (dist <= bestDist)
    {
      bestDist = dist;
      bestModel = topModel;
      bestName = modelName;
      bestContactPointWorld =
        dist <= this->maxDistance ? candidate.contactPointWorld : suctionPose.Pos();
    }
  }

  if (bestModel == gz::sim::kNullEntity)
  {
    std::cerr << "[VacuumGripper][DBG] PostUpdate() sin candidato valido.\n";
    return;
  }

  this->heldModelEntity = bestModel;
  this->heldModelName = bestName;
  this->heldContactPointWorld = bestContactPointWorld;
  this->holding = true;
  this->needsOffsetInit = true;

  gzmsg << "[VacuumGripper] Agarre detectado modelo=["
        << bestName
        << "] dist_contact=" << bestDist
        << " m"
        << (bestDist > this->maxDistance ? " (aceptado por sensor de contacto)" : "")
        << "\n";

  std::cerr << "[VacuumGripper][DBG] PostUpdate()"
            << " holding=true"
            << " needsOffsetInit=true"
            << " modelo=[" << bestName << "]"
            << " entity=" << bestModel
            << "\n";
}

// ---------------------------------------------------------------------------
// PreUpdate:
// Mientras hay agarre:
//   - calcula una vez el offset desde la punta de succión hasta el objeto,
//   - impone WorldPoseCmd cada tick como si el objeto estuviera soldado,
//   - fuerza velocidades lineal/angular a cero.
//
// Esta versión asume que suction_link es el frame/link colocado en la punta
// real de la ventosa, por ejemplo vacuum_suction_tip_link.
// ---------------------------------------------------------------------------
void VacuumGripper::PreUpdate(
  const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &_ecm)
{
  if (!this->configured || _info.paused)
    return;

  std::lock_guard<std::mutex> lock(this->mutex);

  std::cerr << "[VacuumGripper][DBG] PreUpdate()"
            << " vacuumOn=" << this->vacuumOn
            << " holding=" << this->holding
            << " needsOffsetInit=" << this->needsOffsetInit
            << " heldEntity=" << this->heldModelEntity
            << "\n";

  if (!this->vacuumOn)
  {
    this->contactCandidates.clear();
    if (this->holding)
    {
      gzmsg << "[VacuumGripper] Soltando modelo ["
            << this->heldModelName << "]\n";

      std::cerr << "[VacuumGripper][DBG] PreUpdate() vacuumOn=0 -> Release()\n";

      this->Release(&_ecm);
    }

    return;
  }

  if (!this->holding)
    return;

  if (this->heldModelEntity == gz::sim::kNullEntity)
  {
    gzwarn << "[VacuumGripper] heldModelEntity invalido, liberando.\n";
    this->Release(&_ecm);
    return;
  }

  const auto suctionPoseOpt =
      gz::sim::Link(this->suctionLinkEntity).WorldPose(_ecm);

  if (!suctionPoseOpt.has_value())
    return;

  const auto suctionPose = *suctionPoseOpt;

  // Primer tick tras el agarre:
  // Calculamos el offset desde la punta real de succión hasta el origen
  // del modelo agarrado.
  if (this->needsOffsetInit)
  {
    const auto modelPose =
        gz::sim::worldPose(this->heldModelEntity, _ecm);

    const gz::math::Vector3d suctionToModelWorld =
        modelPose.Pos() - suctionPose.Pos();

    const gz::math::Vector3d suctionToModelLocal =
        suctionPose.Rot().RotateVectorReverse(suctionToModelWorld);

    const gz::math::Quaterniond modelRotFromSuction =
        suctionPose.Rot().Inverse() * modelPose.Rot();

    this->heldOffsetFromSuction = gz::math::Pose3d(
        suctionToModelLocal,
        modelRotFromSuction);

    this->needsOffsetInit = false;

    std::cerr << "[VacuumGripper][DBG] PreUpdate() offset inicializado."
              << " modelPose=("
              << modelPose.Pos().X() << ","
              << modelPose.Pos().Y() << ","
              << modelPose.Pos().Z() << ")"
              << " suctionTip=("
              << suctionPose.Pos().X() << ","
              << suctionPose.Pos().Y() << ","
              << suctionPose.Pos().Z() << ")"
              << " contactPoint=("
              << this->heldContactPointWorld.X() << ","
              << this->heldContactPointWorld.Y() << ","
              << this->heldContactPointWorld.Z() << ")"
              << " suctionToModelLocal=("
              << suctionToModelLocal.X() << ","
              << suctionToModelLocal.Y() << ","
              << suctionToModelLocal.Z() << ")"
              << "\n";

    gzmsg << "[VacuumGripper] Agarrado modelo ["
          << this->heldModelName
          << "] offset fijado respecto a suction_link.\n";
  }

  const auto targetPose = suctionPose * this->heldOffsetFromSuction;
  const gz::math::Vector3d zeroVel = gz::math::Vector3d::Zero;

  std::cerr << "[VacuumGripper][DBG] PreUpdate() targetPos=("
            << targetPose.Pos().X() << ","
            << targetPose.Pos().Y() << ","
            << targetPose.Pos().Z() << ")"
            << " modo=kinematic_world_pose_cmd\n";

  auto *poseComp =
      _ecm.Component<gz::sim::components::WorldPoseCmd>(
        this->heldModelEntity);

  if (!poseComp)
  {
    _ecm.CreateComponent(
      this->heldModelEntity,
      gz::sim::components::WorldPoseCmd(targetPose));

    std::cerr << "[VacuumGripper][DBG] PreUpdate() WorldPoseCmd creado.\n";
  }
  else
  {
    poseComp->SetData(targetPose, [](auto &, const auto &) { return true; });
  }

  auto *linComp =
      _ecm.Component<gz::sim::components::LinearVelocityCmd>(
        this->heldModelEntity);

  if (!linComp)
  {
    _ecm.CreateComponent(
      this->heldModelEntity,
      gz::sim::components::LinearVelocityCmd(zeroVel));

    std::cerr << "[VacuumGripper][DBG] PreUpdate() LinearVelocityCmd creado.\n";
  }
  else
  {
    linComp->SetData(zeroVel, [](auto &, const auto &) { return true; });
  }

  auto *angComp =
      _ecm.Component<gz::sim::components::AngularVelocityCmd>(
        this->heldModelEntity);

  if (!angComp)
  {
    _ecm.CreateComponent(
      this->heldModelEntity,
      gz::sim::components::AngularVelocityCmd(zeroVel));

    std::cerr << "[VacuumGripper][DBG] PreUpdate() AngularVelocityCmd creado.\n";
  }
  else
  {
    angComp->SetData(zeroVel, [](auto &, const auto &) { return true; });
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
