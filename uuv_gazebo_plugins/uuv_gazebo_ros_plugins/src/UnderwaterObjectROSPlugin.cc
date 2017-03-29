// Copyright (c) 2016 The UUV Simulator Authors.
// All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <uuv_gazebo_ros_plugins/UnderwaterObjectROSPlugin.hh>

#include <gazebo/physics/Base.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Link.hh>

namespace uuv_simulator_ros
{
/////////////////////////////////////////////////
UnderwaterObjectROSPlugin::UnderwaterObjectROSPlugin()
{
}

/////////////////////////////////////////////////
UnderwaterObjectROSPlugin::~UnderwaterObjectROSPlugin()
{
  this->rosNode->shutdown();
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::Load(gazebo::physics::ModelPtr _parent,
                             sdf::ElementPtr _sdf)
{
  if (!ros::isInitialized())
  {
    gzerr << "Not loading plugin since ROS has not been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
    return;
  }

  this->rosNode.reset(new ros::NodeHandle(""));

  try
  {
    UnderwaterObjectPlugin::Load(_parent, _sdf);
  }
  catch(gazebo::common::Exception &_e)
  {
    gzerr << "Error loading plugin."
          << "Please ensure that your model is correct."
          << '\n';
    return;
  }

  this->subLocalCurVel = this->rosNode->subscribe<geometry_msgs::Vector3>(
    _parent->GetName() + "/current_velocity", 10,
    boost::bind(&UnderwaterObjectROSPlugin::UpdateLocalCurrentVelocity,
    this, _1));

  this->services["set_use_global_current_velocity"] =
    this->rosNode->advertiseService(
      _parent->GetName() + "/set_use_global_current_velocity",
      &UnderwaterObjectROSPlugin::SetUseGlobalCurrentVel, this);

  this->services["get_uuv_link_properties"] =
    this->rosNode->advertiseService(_parent->GetName() + \
    "/get_uuv_link_properties",
    &UnderwaterObjectROSPlugin::GetUUVLinkProperties, this);

  this->rosHydroPub["current_velocity_marker"] =
    this->rosNode->advertise<visualization_msgs::Marker>
    (_parent->GetName() + "/current_velocity_marker", 0);

  this->rosHydroPub["using_global_current_velocity"] =
    this->rosNode->advertise<std_msgs::Bool>
    (_parent->GetName() + "/using_global_current_velocity", 0);
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::Init()
{
  UnderwaterObjectPlugin::Init();
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::Reset()
{
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::InitDebug(gazebo::physics::LinkPtr _link,
  gazebo::HydrodynamicModelPtr _hydro)
{
  UnderwaterObjectPlugin::InitDebug(_link, _hydro);

  // Publish the stamped wrench topics if the debug flag is on
  for (std::map<std::string,
    gazebo::transport::PublisherPtr>::iterator it = this->hydroPub.begin();
    it != this->hydroPub.end(); ++it)
  {
    this->rosHydroPub[it->first] =
      this->rosNode->advertise<geometry_msgs::WrenchStamped>(
        it->second->GetTopic(), 10);
      gzmsg << "ROS TOPIC: " << it->second->GetTopic() << std::endl;
  }
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::PublishRestoringForce(
  gazebo::physics::LinkPtr _link)
{
  // Call base class method
  UnderwaterObjectPlugin::PublishRestoringForce(_link);

  // Publish data in a ROS topic
  if (this->models.count(_link))
  {
    if (!this->models[_link]->GetDebugFlag())
      return;

    gazebo::math::Vector3 restoring = this->models[_link]->GetStoredVector(
      RESTORING_FORCE);

    geometry_msgs::WrenchStamped msg;
    this->GenWrenchMsg(restoring,
      gazebo::math::Vector3(0, 0, 0), msg);
    this->rosHydroPub[_link->GetName() + "/restoring"].publish(msg);
  }
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::PublishCurrentVelocityMarker()
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time();
  marker.ns = this->model->GetName() + "/current_velocity_marker";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  // Creating the arrow marker for the current velocity information
  // (orientation only, magnitude has to be read from the topic)
  if (this->flowVelocity.GetLength() > 0)
  {
    marker.action = visualization_msgs::Marker::ADD;

    gazebo::math::Pose pose = this->model->GetWorldPose();

    double yaw = std::atan2(this->flowVelocity.y, this->flowVelocity.x);
    double pitch = std::atan2(
      this->flowVelocity.z,
      std::sqrt(std::pow(this->flowVelocity.x, 2) +
        std::pow(this->flowVelocity.y, 2)));

    gazebo::math::Quaternion qt(0.0, -pitch, yaw);
    marker.pose.position.x = pose.pos.x;
    marker.pose.position.y = pose.pos.y;
    marker.pose.position.z = pose.pos.z + 1.5;
    marker.pose.orientation.x = qt.x;
    marker.pose.orientation.y = qt.y;
    marker.pose.orientation.z = qt.z;
    marker.pose.orientation.w = qt.w;
    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
  }
  else
  {
    marker.action = visualization_msgs::Marker::DELETE;
  }
  // Publish current velocity RViz marker
  this->rosHydroPub["current_velocity_marker"].publish(marker);
  // Publishing flag for usage of global current velocity
  std_msgs::Bool useGlobalMsg;
  useGlobalMsg.data = this->useGlobalCurrent;
  this->rosHydroPub["using_global_current_velocity"].publish(useGlobalMsg);
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::PublishHydrodynamicWrenches(
  gazebo::physics::LinkPtr _link)
{
  // Call base class method
  UnderwaterObjectPlugin::PublishRestoringForce(_link);

  // Publish data in a ROS topic
  if (this->models.count(_link))
  {
    if (!this->models[_link]->GetDebugFlag())
      return;
    geometry_msgs::WrenchStamped msg;
    gazebo::math::Vector3 force, torque;

    // Publish wrench generated by the acceleration of fluid around the object
    force = this->models[_link]->GetStoredVector(UUV_ADDED_MASS_FORCE);
    torque = this->models[_link]->GetStoredVector(UUV_ADDED_MASS_TORQUE);

    this->GenWrenchMsg(force, torque, msg);
    this->rosHydroPub[_link->GetName() + "/added_mass"].publish(msg);

    // Publish wrench generated by the fluid damping
    force = this->models[_link]->GetStoredVector(UUV_DAMPING_FORCE);
    torque = this->models[_link]->GetStoredVector(UUV_DAMPING_TORQUE);

    this->GenWrenchMsg(force, torque, msg);
    this->rosHydroPub[_link->GetName() + "/damping"].publish(msg);

    // Publish wrench generated by the Coriolis forces
    force = this->models[_link]->GetStoredVector(UUV_ADDED_CORIOLIS_FORCE);
    torque = this->models[_link]->GetStoredVector(UUV_ADDED_CORIOLIS_TORQUE);

    this->GenWrenchMsg(force, torque, msg);
    this->rosHydroPub[_link->GetName() + "/added_coriolis"].publish(msg);
  }
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::GenWrenchMsg(
  gazebo::math::Vector3 _force, gazebo::math::Vector3 _torque,
  geometry_msgs::WrenchStamped &_output)
{
  _output.wrench.force.x = _force.x;
  _output.wrench.force.y = _force.y;
  _output.wrench.force.z = _force.z;

  _output.wrench.torque.x = _torque.x;
  _output.wrench.torque.y = _torque.y;
  _output.wrench.torque.z = _torque.z;

  _output.header.stamp = ros::Time(this->world->GetSimTime().Double());
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::UpdateLocalCurrentVelocity(
  const geometry_msgs::Vector3::ConstPtr &_msg)
{
  if (!this->useGlobalCurrent)
  {
    this->flowVelocity.x = _msg->x;
    this->flowVelocity.y = _msg->y;
    this->flowVelocity.z = _msg->z;
  }
}

/////////////////////////////////////////////////
bool UnderwaterObjectROSPlugin::SetUseGlobalCurrentVel(
  uuv_gazebo_ros_plugins_msgs::SetUseGlobalCurrentVel::Request& _req,
  uuv_gazebo_ros_plugins_msgs::SetUseGlobalCurrentVel::Response& _res)
{
  if (_req.use_global == this->useGlobalCurrent)
    _res.success = true;
  else
  {
    this->useGlobalCurrent = _req.use_global;
    this->flowVelocity.x = 0;
    this->flowVelocity.y = 0;
    this->flowVelocity.z = 0;
    if (this->useGlobalCurrent)
      gzmsg << this->model->GetName() <<
        "::Now using global current velocity" << std::endl;
    else
      gzmsg << this->model->GetName() <<
        "::Using the current velocity under the namespace " <<
        this->model->GetName() << std::endl;
    _res.success = true;
  }
  return true;
}

/////////////////////////////////////////////////
bool UnderwaterObjectROSPlugin::GetUUVLinkProperties(
  uuv_gazebo_ros_plugins_msgs::GetUUVLinkProperties::Request& _req,
  uuv_gazebo_ros_plugins_msgs::GetUUVLinkProperties::Response& _res)
{
  for (std::map<gazebo::physics::LinkPtr,
       gazebo::HydrodynamicModelPtr>::iterator it = this->models.begin();
       it != this->models.end(); ++it)
  {
    gazebo::physics::LinkPtr link = it->first;
    if (link->GetName().compare(_req.link_name) == 0)
    {
      // Set mass
      _res.mass = link->GetInertial()->GetMass();
      // Set inertial tensor coefficients
      _res.ixx = link->GetInertial()->GetIXX();
      _res.ixy = link->GetInertial()->GetIXY();
      _res.ixz = link->GetInertial()->GetIXZ();
      _res.iyy = link->GetInertial()->GetIYY();
      _res.iyz = link->GetInertial()->GetIYZ();
      _res.izz = link->GetInertial()->GetIZZ();
      // Set body volume
      _res.volume = this->models[link]->GetVolume();
      // Set current pose in the Gazebo ENU frame
      gazebo::math::Pose pose = link->GetWorldPose();
      _res.pose.position.x = pose.pos.x;
      _res.pose.position.y = pose.pos.y;
      _res.pose.position.z = pose.pos.z;
      _res.pose.orientation.x = pose.rot.x;
      _res.pose.orientation.y = pose.rot.y;
      _res.pose.orientation.z = pose.rot.z;
      _res.pose.orientation.w = pose.rot.w;
      // Set hydrodynamic parameters
      _res.added_mass = this->models[link]->GetParameterAsVector("added_mass");
      _res.linear_damping = this->models[link]->GetParameterAsVector(
        "linear_damping");
      _res.quad_damping = this->models[link]->GetParameterAsVector(
        "quad_damping");

      // Set scaling factors
      _res.added_mass_scaling = this->models[link]->GetParam(
        "added_mass_scaling");
      _res.linear_damping_scaling = this->models[link]->GetParam(
        "linear_damping_scaling");
      _res.quad_damping_scaling = this->models[link]->GetParam(
        "quad_damping_scaling");

      // Set bounding box properties
      _res.bbox_width = this->models[link]->GetParam("bbox_width");
      _res.bbox_length = this->models[link]->GetParam("bbox_length");
      _res.bbox_height = this->models[link]->GetParam("bbox_height");

      // Set fluid density
      _res.fluid_density = this->models[link]->GetFluidDensity();

      // Set neutrally buoyant flag
      _res.is_neutrally_buoyant = this->models[link]->IsNeutrallyBuoyant();

      // Set center of mass and center of buoyancy
      _res.center_of_buoyancy.x = this->models[link]->GetCoB().x;
      _res.center_of_buoyancy.y = this->models[link]->GetCoB().y;
      _res.center_of_buoyancy.z = this->models[link]->GetCoB().z;

      _res.center_of_mass.x = link->GetInertial()->GetCoG().x;
      _res.center_of_mass.y = link->GetInertial()->GetCoG().y;
      _res.center_of_mass.z = link->GetInertial()->GetCoG().z;

      // Set global current flag
      _res.using_global_current = this->useGlobalCurrent;

      _res.success = true;
      _res.status_message = this->model->GetName() + "::" + link->GetName() + \
        " link found.";
      return true;
    }
  }
  _res.success = false;
  _res.status_message =
    "Invalid link name or link doesn't have a hydrodynamic model";
  return true;
}

GZ_REGISTER_MODEL_PLUGIN(UnderwaterObjectROSPlugin)
}
