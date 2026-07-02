// Gazebo (gz-sim8) system plugin: runtime "payload drop" on base_link, for
// testing the MHE/NMPC mass-estimation loop without ever putting an
// independent dynamic rigid body in the world (that path was tried first -
// see model.sdf's history comment - and reliably made the drone go into a
// violent, unrecoverable attitude oscillation from the moment it tried to
// climb, regardless of how the extra body was spawned or where it sat).
//
// How the drop is emulated (and why it is NOT a real inertial swap):
// gz-sim Harmonic has no way to change a link's mass at runtime such that
// the DART physics integrator picks it up - SetComponentData<Inertial> only
// updates the ECM component (so `gz model` reads the new value and it LOOKS
// applied) while the integrator keeps the mass from model-load time
// (gz-sim issue #2733). An earlier version of this plugin did exactly that
// SetInertial-on-drop and it was silently a no-op: 0.1 s diagnostic logs
// showed zero disturbance to thrust/vz at the drop instant, i.e. the drone's
// dynamic mass never changed and the MHE (correctly) kept estimating the
// constant true mass.
//
// So instead: Configure sets base_link's inertial to LOADED (2.5 kg) once,
// before physics init, where it IS honored. On /payload/drop_mass the plugin
// leaves the inertial alone and, every sim step, applies a constant upward
// world-frame force (kLoadedMass - kEmptyMass)*g on base_link via
// Link::AddWorldForce (ExternalWorldWrenchCmd, which DOES take effect at
// runtime). The drone then hovers at the empty-mass thrust, so the MHE's
// translational mass estimate steps down to the empty mass - the behavior
// we actually want to test. Only the translational mass is emulated, not the
// rotational inertia, which is acceptable because the MHE only estimates the
// mass scalar.
#ifndef MASS_CHANGER_HH_
#define MASS_CHANGER_HH_

#include <atomic>

#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/empty.pb.h>

namespace mass_changer
{

class MassChanger:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
  public: MassChanger() = default;
  public: ~MassChanger() override = default;

  public: void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &_eventMgr) override;

  public: void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override;

  // Called on the gz-transport receive thread - must stay cheap and must
  // not touch the ECM directly (no thread safety guarantee there). Only
  // sets a flag; the actual component write happens in PreUpdate, which
  // runs on the simulation thread.
  private: void OnDropMsg(const gz::msgs::Empty &_msg);

  private: void SetInertial(
      gz::sim::EntityComponentManager &_ecm,
      double _mass, double _ixx, double _iyy, double _izz);

  private: gz::sim::Entity baseLinkEntity{gz::sim::kNullEntity};
  private: gz::transport::Node node;
  private: std::atomic<bool> dropRequested{false};
  private: bool dropApplied{false};
};

}  // namespace mass_changer

#endif
