/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include <gz/msgs/param.pb.h>
#include <chrono>
#include <mutex>
#include <string>
#include <gz/common/Profiler.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <sdf/sdf.hh>

#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/World.hh"
#include "gz/transport/Node.hh"
#include "gz/physics.hh"

#include "SurfacePlace.hh"
#include "Wavefield.hh"

using namespace gz;
using namespace vrx;

/// \brief Private SurfacePlace data class.
class vrx::SurfacePlace::Implementation
{
  /// \brief Parse the points via SDF.
  /// \param[in] _sdf Pointer to the SDF.
  public: void ParsePoints(const std::shared_ptr<const sdf::Element> &_sdf);

  /// \brief Callback for receiving wave field updates.
  /// \param[in] _msg The message containing all the wave field parameters.
  public: void OnWavefield(const msgs::Param &_msg);

  /// \brief The link entity.
  public: sim::Link link{sim::kNullEntity};
  public: sim::Link link0{sim::kNullEntity};
  public: sim::Link link1{sim::kNullEntity};
  public: sim::Link link2{sim::kNullEntity};
  public: sim::Link link3{sim::kNullEntity};
  public: sim::Link link4{sim::kNullEntity};
  public: sim::Link link5{sim::kNullEntity};

  /// \brief Vessel length [m].
  public: double HullHeigth = 0.283;

  /// \brief Demi-hull radius [m].
  public: double hullRadius = 0.05;

  /// \brief Fluid height [m].
  public: double fluidLevel = 0;

  /// \brief Fluid density [kg/m^3].
  public: double fluidDensity = 997.7735;

  /// \brief The world's gravity [m/s^2].
  public: math::Vector3d gravity;

  /// \brief The points where the plugin applies forces. These points are
  /// relative to the link paramter's origin. Note that we don't check that the
  /// points are contained within the hull. You should pass reasonable points.
  public: std::vector<math::Vector3d> points;

  /// \brief The wavefield.
  public: Wavefield wavefield;

  /// \brief A node for receiving wavefield updates.
  public: transport::Node node;

  /// \brief Mutex to protect the wavefield.
  public: std::mutex mutex;

  /// \brief Values for differentiation
  public: double old_z = 0;
  public: double old_2_z = 0;
  public: double dot_z = 0;
  public: double dot_z_old = 0;

  public: double old_z_0 = 0;
  public: double old_2_z_0 = 0;
  public: double dot_z_0 = 0;
  public: double dot_z_old_0 = 0;

  public: double old_z_1 = 0;
  public: double old_2_z_1 = 0;
  public: double dot_z_1 = 0;
  public: double dot_z_old_1 = 0;

  public: double old_z_2 = 0;
  public: double old_2_z_2 = 0;
  public: double dot_z_2 = 0;
  public: double dot_z_old_2 = 0;

  public: double old_z_3 = 0;
  public: double old_2_z_3 = 0;
  public: double dot_z_3 = 0;
  public: double dot_z_old_3 = 0;

  public: double old_z_4 = 0;
  public: double old_2_z_4 = 0;
  public: double dot_z_4 = 0;
  public: double dot_z_old_4 = 0;

  public: double old_z_5 = 0;
  public: double old_2_z_5 = 0;
  public: double dot_z_5 = 0;
  public: double dot_z_old_5 = 0;
};

//////////////////////////////////////////////////
void SurfacePlace::Implementation::ParsePoints(
  const std::shared_ptr<const sdf::Element> &_sdf)
{
  if (!_sdf->HasElement("points"))
    return;

  auto ptr = const_cast<sdf::Element *>(_sdf.get());
  auto sdfPoints = ptr->GetElement("points");

  // We need at least one point.
  if (!sdfPoints->HasElement("point"))
    gzerr << "Unable to find <points><point> element in SDF." << std::endl;

  auto pointElem = sdfPoints->GetElement("point");

  // Parse a new point.
  while (pointElem)
  {
    math::Vector3d point;
    pointElem->GetValue()->Get<math::Vector3d>(point);
    this->points.push_back(point);

    // Parse the next point.
    pointElem = pointElem->GetNextElement("point");
  }
}

//////////////////////////////////////////////////
void SurfacePlace::Implementation::OnWavefield(const msgs::Param &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->wavefield.Load(_msg);
}

//////////////////////////////////////////////////
SurfacePlace::SurfacePlace()
  : System(), dataPtr(utils::MakeUniqueImpl<Implementation>())
{
}

//////////////////////////////////////////////////
void SurfacePlace::Configure(const sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    sim::EntityComponentManager &_ecm,
    sim::EventManager &/*_eventMgr*/)
{
  // Parse required elements.
  if (!_sdf->HasElement("link_name"))
  {
    gzerr << "No <link_name> specified" << std::endl;
    return;
  }

  sim::Model model(_entity);
  std::string linkName = _sdf->Get<std::string>("link_name");
  this->dataPtr->link = sim::Link(model.LinkByName(_ecm, linkName));
  if (!this->dataPtr->link.Valid(_ecm))
  {
    gzerr << "Could not find link named [" << linkName
           << "] in model" << std::endl;
    return;
  }
  this->dataPtr->link0 = sim::Link(model.LinkByName(_ecm, "rotor_0"));
  this->dataPtr->link1 = sim::Link(model.LinkByName(_ecm, "rotor_1"));
  this->dataPtr->link2 = sim::Link(model.LinkByName(_ecm, "rotor_2"));
  this->dataPtr->link3 = sim::Link(model.LinkByName(_ecm, "rotor_3"));
  this->dataPtr->link4 = sim::Link(model.LinkByName(_ecm, "rotor_4"));
  this->dataPtr->link5 = sim::Link(model.LinkByName(_ecm, "rotor_5"));

  // Optional parameters.
  // Although some of these parameters are required in this plugin, a potential
  // derived plugin might not need them. Make sure that the default values are
  // reasonable.
  if (_sdf->HasElement("hell_height"))
  {
    this->dataPtr->HullHeigth = _sdf->Get<double>("hell_height");
  }

  if (_sdf->HasElement("hull_radius"))
  {
    this->dataPtr->hullRadius = _sdf->Get<double>("hull_radius");
  }

  if (_sdf->HasElement("fluid_level"))
  {
    this->dataPtr->fluidLevel = _sdf->Get<double>("fluid_level");
  }

  if (_sdf->HasElement("fluid_density"))
  {
    this->dataPtr->fluidDensity = _sdf->Get<double>("fluid_density");
  }

  // Parse the optional <points> element.
  this->dataPtr->ParsePoints(_sdf);

  // Get the gravity from the world.
  auto worldEntity = sim::worldEntity(_ecm);
  auto world = sim::World(worldEntity);
  auto gravityOpt = world.Gravity(_ecm);
  if (!gravityOpt)
  {
    gzerr << "Unable to get the gravity from the world" << std::endl;
    return;
  }
  this->dataPtr->gravity = *gravityOpt;

  // Wavefield
  this->dataPtr->wavefield.Load(_sdf);

  gzdbg << "SurfacePlace plugin successfully configured with the following "
         << "parameters:" << std::endl;
  gzdbg << "  <link_name>: " << linkName << std::endl;
  gzdbg << "  <vehicle_length>: " << this->dataPtr->HullHeigth << std::endl;
  gzdbg << "  <hull_radius>: " << this->dataPtr->hullRadius << std::endl;
  gzdbg << "  <fluid_level>: " << this->dataPtr->fluidLevel << std::endl;
  gzdbg << "  <fluid_density>: " << this->dataPtr->fluidDensity << std::endl;
  gzdbg << "  <points>:" << std::endl;
  for (const auto &p : this->dataPtr->points)
    gzdbg << "    [" << p << "]" << std::endl;

  // Subscribe to receive wavefield parameters.
  this->dataPtr->node.Subscribe(this->dataPtr->wavefield.Topic(),
    &SurfacePlace::Implementation::OnWavefield, this->dataPtr.get());
}

//////////////////////////////////////////////////
void SurfacePlace::PreUpdate(const sim::UpdateInfo &_info,
    sim::EntityComponentManager &_ecm)
{
  GZ_PROFILE("SurfacePlace::PreUpdate");

  if (_info.paused)
    return;

  // Vehicle frame transform.
  const auto kPose = this->dataPtr->link.WorldPose(_ecm);
  if (!kPose)
  {
    gzerr << "Unable to get world pose from link ["
           << this->dataPtr->link.Entity() << "]" << std::endl;
    return;
  }
  const math::Vector3d kEuler = (*kPose).Rot().Euler();
  math::Quaternion vq(kEuler.X(), kEuler.Y(), kEuler.Z());
  const auto pos_z = (*this->dataPtr->link.WorldPose(_ecm)).Pos().Z();
  const auto pos_z0 = (*this->dataPtr->link0.WorldPose(_ecm)).Pos().Z();
  const auto pos_z1 = (*this->dataPtr->link1.WorldPose(_ecm)).Pos().Z();
  const auto pos_z2 = (*this->dataPtr->link2.WorldPose(_ecm)).Pos().Z();
  const auto pos_z3 = (*this->dataPtr->link3.WorldPose(_ecm)).Pos().Z();
  const auto pos_z4 = (*this->dataPtr->link4.WorldPose(_ecm)).Pos().Z();
  const auto pos_z5 = (*this->dataPtr->link5.WorldPose(_ecm)).Pos().Z();

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  const int dmp = 5;
  const int side_dmp = 5;

  for (auto const &bpnt : this->dataPtr->points)
  {
    // Transform from vessel to fluid/world frame.
    const math::Vector3d kBpntW = vq * bpnt;

    // Vertical location of boat grid point in world frame.
    const float kDdz = (*kPose).Pos().Z() + kBpntW.Z();

    // World location of grid point.
    math::Vector3d point;
    point.X() = (*kPose).Pos().X() + kBpntW.X();
    point.Y() = (*kPose).Pos().Y() + kBpntW.Y();

    // Compute the depth at the grid point.
    double simTime = std::chrono::duration<double>(_info.simTime).count();
    double depth = this->dataPtr->wavefield.ComputeDepthDirectly(point, simTime);

    // Vertical wave displacement.
    double dz = depth + point.Z();

    // Total z location of boat grid point relative to fluid SurfacePlace.
    double deltaZ = (this->dataPtr->fluidLevel + dz) - kDdz;
    // Enforce only upward buoy force
    deltaZ = std::max(deltaZ, 0.0);
    deltaZ = std::min(deltaZ, this->dataPtr->HullHeigth);

    float kBuoyForce = 0;
    //gzdbg<<(pos_z-this->dataPtr->old_z)/0.001<<std::endl;
    //gzdbg<<this->dataPtr->dot_z_old<<std::endl;
    this->dataPtr->dot_z = (pos_z-this->dataPtr->old_2_z)/0.002/4;
    this->dataPtr->dot_z_0 = (pos_z0-this->dataPtr->old_2_z_0)/0.002/4;
    this->dataPtr->dot_z_1 = (pos_z1-this->dataPtr->old_2_z_1)/0.002/4;
    this->dataPtr->dot_z_2 = (pos_z2-this->dataPtr->old_2_z_2)/0.002/4;
    this->dataPtr->dot_z_3 = (pos_z3-this->dataPtr->old_2_z_3)/0.002/4;
    this->dataPtr->dot_z_4 = (pos_z4-this->dataPtr->old_2_z_4)/0.002/4;
    this->dataPtr->dot_z_5 = (pos_z5-this->dataPtr->old_2_z_5)/0.002/4;

    if(this->dataPtr->points.size()>1)
    {
      gzdbg<<"in base"<<std::endl;
      if(deltaZ !=0)
      {
        gzdbg<<"touchdown"<<std::endl;
        kBuoyForce =
          this->CylinderVolume(this->dataPtr->hullRadius, deltaZ) / 
            this->dataPtr->points.size() * -this->dataPtr->gravity.Z() * this->dataPtr->fluidDensity - dmp * this->dataPtr->dot_z_old;
      }
      else
      {
        gzdbg<<"__________detached__________"<<std::endl;
        kBuoyForce = 0;
      }
      gzdbg<<kBuoyForce<<std::endl;
      if(deltaZ > this->dataPtr->HullHeigth)
        gzdbg<<"buoyant part completely submerged"<<std::endl;
    }
    else if(bpnt==math::Vector3d(0, 0.8, 0))
    {
      //gzdbg<<"in 0"<<std::endl;
      if(deltaZ !=0)
      {
        gzdbg<<"touchdown"<<std::endl;
        kBuoyForce =
          this->CylinderVolume(this->dataPtr->hullRadius, deltaZ) / 
            this->dataPtr->points.size() * -this->dataPtr->gravity.Z() * this->dataPtr->fluidDensity - side_dmp * this->dataPtr->dot_z_old_0;
      }
      else
      {
        gzdbg<<"__________detached__________"<<std::endl;
        kBuoyForce = 0;
      }
      gzdbg<<kBuoyForce<<std::endl;
      if(deltaZ > this->dataPtr->HullHeigth)
        gzdbg<<"buoyant part completely submerged"<<std::endl;
    }
    else if(bpnt==math::Vector3d(0.692, 0.4, 0))
    {
      gzdbg<<"in 1"<<std::endl;
      if(deltaZ !=0)
      {
        //gzdbg<<"touchdown"<<std::endl;
        kBuoyForce =
          this->CylinderVolume(this->dataPtr->hullRadius, deltaZ) / 
            this->dataPtr->points.size() * -this->dataPtr->gravity.Z() * this->dataPtr->fluidDensity - side_dmp * this->dataPtr->dot_z_old_1;
      }
      else
      {
        gzdbg<<"__________detached__________"<<std::endl;
        kBuoyForce = 0;
      }
      gzdbg<<kBuoyForce<<std::endl;
      if(deltaZ > this->dataPtr->HullHeigth)
        gzdbg<<"buoyant part completely submerged"<<std::endl;
    }
    else if(bpnt==math::Vector3d(0.692, -0.4, 0))
    {
      gzdbg<<"in 2"<<std::endl;
      if(deltaZ !=0)
      {
        gzdbg<<"touchdown"<<std::endl;
        kBuoyForce =
          this->CylinderVolume(this->dataPtr->hullRadius, deltaZ) / 
            this->dataPtr->points.size() * -this->dataPtr->gravity.Z() * this->dataPtr->fluidDensity - side_dmp * this->dataPtr->dot_z_old_2;
      }
      else
      {
        gzdbg<<"__________detached__________"<<std::endl;
        kBuoyForce = 0;
      }
      gzdbg<<kBuoyForce<<std::endl;
      if(deltaZ > this->dataPtr->HullHeigth)
        gzdbg<<"buoyant part completely submerged"<<std::endl;
    }
    else if(bpnt==math::Vector3d(0, -0.8, 0))
    {
      gzdbg<<"in 3"<<std::endl;
      if(deltaZ !=0)
      {
        gzdbg<<"touchdown"<<std::endl;
        kBuoyForce =
          this->CylinderVolume(this->dataPtr->hullRadius, deltaZ) / 
            this->dataPtr->points.size() * -this->dataPtr->gravity.Z() * this->dataPtr->fluidDensity - side_dmp * this->dataPtr->dot_z_old_3;
      }
      else
      {
        gzdbg<<"__________detached__________"<<std::endl;
        kBuoyForce = 0;
      }
      gzdbg<<kBuoyForce<<std::endl;
      if(deltaZ > this->dataPtr->HullHeigth)
        gzdbg<<"buoyant part completely submerged"<<std::endl;
    }
    else if(bpnt==math::Vector3d(-0.692, -0.4, 0))
    {
      gzdbg<<"in 4"<<std::endl;
      if(deltaZ !=0)
      {
        gzdbg<<"touchdown"<<std::endl;
        kBuoyForce =
          this->CylinderVolume(this->dataPtr->hullRadius, deltaZ) / 
            this->dataPtr->points.size() * -this->dataPtr->gravity.Z() * this->dataPtr->fluidDensity - side_dmp * this->dataPtr->dot_z_old_4;
      }
      else
      {
        gzdbg<<"__________detached__________"<<std::endl;
        kBuoyForce = 0;
      }
      gzdbg<<kBuoyForce<<std::endl;
      if(deltaZ > this->dataPtr->HullHeigth)
        gzdbg<<"buoyant part completely submerged"<<std::endl;
    }
    else if(bpnt==math::Vector3d(-0.692, 0.4, 0))
    {
      gzdbg<<"in 5"<<std::endl;
      if(deltaZ !=0)
      {
        gzdbg<<"touchdown"<<std::endl;
        kBuoyForce =
          this->CylinderVolume(this->dataPtr->hullRadius, deltaZ) / 
            this->dataPtr->points.size() * -this->dataPtr->gravity.Z() * this->dataPtr->fluidDensity - side_dmp * this->dataPtr->dot_z_old_5;
      }
      else
      {
        gzdbg<<"__________detached__________"<<std::endl;
        kBuoyForce = 0;
      }
      gzdbg<<kBuoyForce<<std::endl;
      if(deltaZ > this->dataPtr->HullHeigth)
        gzdbg<<"buoyant part completely submerged"<<std::endl;
    }
    // Apply force at the point.
    // Position is in the link frame and force is in world frame.
    this->dataPtr->link.AddWorldForce(_ecm,
      math::Vector3d(0, 0, kBuoyForce),
      bpnt);

    // Debug output:
    // gzdbg << bpnt.X() << "," << bpnt.Y() << "," << bpnt.Z() << std::endl;
    // gzdbg << "depth: " << depth << std::endl;
    // gzdbg << "dz: " << dz << std::endl;
    // gzdbg << "kDdz: " << kDdz << std::endl;
    // gzdbg << "deltaZ: " << deltaZ << std::endl;
    // gzdbg << "hull radius: " << this->dataPtr->hullRadius << std::endl;
    // gzdbg << "vehicle length: " << this->dataPtr->HullHeigth << std::endl;
    // gzdbg << "gravity z: " << -this->dataPtr->gravity.Z() << std::endl;
    // gzdbg << "fluid density: " << this->dataPtr->fluidDensity << std::endl;
    // gzdbg << "Force: " << kBuoyForce << std::endl << std::endl;
  }
  
  this->dataPtr->old_2_z = this->dataPtr->old_z;
  this->dataPtr->old_z = pos_z;
  this->dataPtr->dot_z_old = this->dataPtr->dot_z;

  this->dataPtr->old_2_z_0 = this->dataPtr->old_z_0;
  this->dataPtr->old_z_0 = pos_z0;
  this->dataPtr->dot_z_old_0 = this->dataPtr->dot_z_0;

  this->dataPtr->old_2_z_1 = this->dataPtr->old_z_1;
  this->dataPtr->old_z_1 = pos_z1;
  this->dataPtr->dot_z_old_1 = this->dataPtr->dot_z_1;

  this->dataPtr->old_2_z_2 = this->dataPtr->old_z_2;
  this->dataPtr->old_z_2 = pos_z2;
  this->dataPtr->dot_z_old_2 = this->dataPtr->dot_z_2;

  this->dataPtr->old_2_z_3 = this->dataPtr->old_z_3;
  this->dataPtr->old_z_3 = pos_z3;
  this->dataPtr->dot_z_old_3 = this->dataPtr->dot_z_3;

  this->dataPtr->old_2_z_4 = this->dataPtr->old_z_4;
  this->dataPtr->old_z_4 = pos_z4;
  this->dataPtr->dot_z_old_4 = this->dataPtr->dot_z_4;

  this->dataPtr->old_2_z_5 = this->dataPtr->old_z_5;
  this->dataPtr->old_z_5 = pos_z5;
  this->dataPtr->dot_z_old_5 = this->dataPtr->dot_z_5;
  
}

//////////////////////////////////////////////////
math::Vector3d SurfacePlace::Gravity() const
{
  return this->dataPtr->gravity;
}

//////////////////////////////////////////////////
double SurfacePlace::HullHeigth() const
{
  return this->dataPtr->HullHeigth;
}

//////////////////////////////////////////////////
double SurfacePlace::HullRadius() const
{
  return this->dataPtr->hullRadius;
}

//////////////////////////////////////////////////
double SurfacePlace::FluidDensity() const
{
  return this->dataPtr->fluidDensity;
}

//////////////////////////////////////////////////
double SurfacePlace::CylinderVolume(double _r, double _h) const
{
  return 3.14*_r*_r*_h;
}

GZ_ADD_PLUGIN(SurfacePlace,
              sim::System,
              SurfacePlace::ISystemConfigure,
              SurfacePlace::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(vrx::SurfacePlace,
                    "vrx::SurfacePlace")
