#include "MassChanger.hh"

#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Name.hh>
#include <gz/common/Console.hh>
#include <gz/math/Inertial.hh>
#include <gz/math/MassMatrix3.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

using namespace mass_changer;

// Loaded: base 2.0 kg + 0.5 kg payload, CoM assumed coincident with
// base_link origin (no parallel-axis offset - see model.sdf comment for
// why: MHE here only estimates the mass scalar, not CoM offset/inertia,
// so an offset would just add unmodeled torque coupling with no upside
// for what this rig is testing). Payload inertia added as a 0.15m-side
// cube about its own CoM (I = m*s^2/6), same box size as before, just a
// lighter payload mass for this test.
static constexpr double kLoadedMass = 2.5;
static constexpr double kLoadedIxx  = 0.02354167;
static constexpr double kLoadedIyy  = 0.02354167;
static constexpr double kLoadedIzz  = 0.04187500;

// Empty: base_link alone, copied from x500_base/model.sdf. NOTE: with the
// wrench-based drop (see below), the inertia never actually changes at
// runtime - these values are kept only for reference / a possible future
// switch back to a real inertial swap. The physical inertial mass stays at
// kLoadedMass (2.5 kg) for the whole flight.
static constexpr double kEmptyMass = 2.0;
static constexpr double kEmptyIxx  = 0.02166667;
static constexpr double kEmptyIyy  = 0.02166667;
static constexpr double kEmptyIzz  = 0.04;

// World gravity magnitude - matches <gravity>0 0 -9.8</gravity> in every PX4
// gz world (worlds/default.sdf etc). Used to size the drop force so it
// exactly cancels the "removed" payload's weight.
static constexpr double kGravity = 9.8;

// Runtime SetComponentData<Inertial> is a no-op for the DART physics engine
// in gz-sim Harmonic (gz-sim issue #2733: the ECM component changes and
// `gz model` reads the new value, but the integrator keeps the mass it saw
// at model-load time). So instead of swapping the inertial on drop, we hold
// the inertial fixed and emulate the payload change with a constant
// world-frame Z force of -delta_kg*g (delta<0 drop -> upward force,
// delta>0 grasp -> downward force). At hover this makes the required thrust
// settle at (m_true + delta)*g - exactly the effective mass the MHE's
// translational (thrust/accel) estimate observes. Caveat: the rotational
// inertia is unchanged, so this only emulates the translational mass step,
// not a full inertia change - fine here, the MHE only estimates the mass
// scalar. delta_kg comes from env MASS_CHANGER_DELTA_KG (default -0.5,
// byte-identical to the pre-parameterization behavior).

//////////////////////////////////////////////////
void MassChanger::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> & /*_sdf*/,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager & /*_eventMgr*/)
{
  gz::sim::Model model(_entity);
  if (!model.Valid(_ecm))
  {
    gzerr << "[MassChanger] Configure: parent entity is not a valid model\n";
    return;
  }

  this->baseLinkEntity = model.LinkByName(_ecm, "base_link");
  if (this->baseLinkEntity == gz::sim::kNullEntity)
  {
    gzerr << "[MassChanger] Configure: could not find link 'base_link' "
          << "on model '" << model.Name(_ecm) << "'\n";
    return;
  }

  this->SetInertial(_ecm, kLoadedMass, kLoadedIxx, kLoadedIyy, kLoadedIzz);

  if (const char *env = std::getenv("MASS_CHANGER_DELTA_KG"))
  {
    try
    {
      this->deltaKg = std::stod(env);
    }
    catch (const std::exception &)
    {
      gzerr << "[MassChanger] bad MASS_CHANGER_DELTA_KG='" << env
            << "', keeping default " << this->deltaKg << " kg\n";
    }
  }
  this->forceZ = -this->deltaKg * kGravity;
  gzmsg << "[MassChanger] Configured: base_link set to LOADED ("
        << kLoadedMass << " kg), delta on trigger = " << this->deltaKg
        << " kg (force Z " << this->forceZ << " N)\n";

  this->node.Subscribe(
      "/payload/drop_mass", &MassChanger::OnDropMsg, this);
}

//////////////////////////////////////////////////
void MassChanger::OnDropMsg(const gz::msgs::Empty & /*_msg*/)
{
  // Transport receive thread - just latch a flag, the actual ECM write
  // happens on the simulation thread in PreUpdate.
  this->dropRequested = true;
}

//////////////////////////////////////////////////
void MassChanger::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused || !this->dropRequested ||
      this->baseLinkEntity == gz::sim::kNullEntity)
    return;

  // Log once, the first sim step after the drop message arrives.
  if (!this->dropApplied)
  {
    this->dropApplied = true;
    gzmsg << "[MassChanger] Mass step applied: emulating "
          << this->deltaKg << " kg via a constant " << this->forceZ
          << " N world-Z force on base_link (inertial mass stays "
          << kLoadedMass << " kg, see class comment) at sim_time="
          << std::chrono::duration<double>(_info.simTime).count() << "s\n";
  }

  // ExternalWorldWrenchCmd is consumed and cleared by the physics system
  // every step (Link::AddWorldForce is documented as "applied for one
  // simulation step"), so the force must be re-added every PreUpdate to stay
  // constant. Applied at the CoM (our inertial's CoM is at the link origin),
  // so it contributes no torque - a pure translational weight offset.
  gz::sim::Link baseLink(this->baseLinkEntity);
  baseLink.AddWorldForce(_ecm, gz::math::Vector3d(0.0, 0.0, this->forceZ));
}

//////////////////////////////////////////////////
void MassChanger::SetInertial(
    gz::sim::EntityComponentManager &_ecm,
    double _mass, double _ixx, double _iyy, double _izz)
{
  if (this->baseLinkEntity == gz::sim::kNullEntity)
    return;

  gz::math::MassMatrix3d massMatrix(
      _mass,
      gz::math::Vector3d(_ixx, _iyy, _izz),
      gz::math::Vector3d(0, 0, 0));
  gz::math::Inertiald inertial(massMatrix, gz::math::Pose3d::Zero);

  _ecm.SetComponentData<gz::sim::components::Inertial>(
      this->baseLinkEntity, inertial);
  // Component was changed out from under the physics engine's cached
  // state - mark it as a one-shot change so downstream systems (physics)
  // pick it up on the next step rather than treating it as unchanged.
  _ecm.SetChanged(this->baseLinkEntity,
      gz::sim::components::Inertial::typeId,
      gz::sim::ComponentState::OneTimeChange);
}

GZ_ADD_PLUGIN(
    mass_changer::MassChanger,
    gz::sim::System,
    mass_changer::MassChanger::ISystemConfigure,
    mass_changer::MassChanger::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(mass_changer::MassChanger, "mass_changer::MassChanger")
