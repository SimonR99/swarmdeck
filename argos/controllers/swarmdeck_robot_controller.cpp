/**
 * @file swarmdeck_robot_controller.cpp
 */

#include "swarmdeck_robot_controller.h"

#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_imu_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_odometry_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_photorealistic_camera_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_photorealistic_lidar_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>

#include <algorithm>
#include <cmath>

/****************************************/
/****************************************/

CSwarmdeckRobotController::CSwarmdeckRobotController() {}

/****************************************/
/****************************************/

void CSwarmdeckRobotController::Init(TConfigurationNode& t_tree) {
   /* Actuators */
   try {
      m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   }
   catch(CARGoSException& ex) {
      LOGERR << "[CSwarmdeckRobotController] Warning: No differential_steering actuator: "
             << ex.what() << std::endl;
      m_pcWheels = nullptr;
   }

   /* Sensors (optional / resilient lookups) */
   try {
      m_pcPositioning = GetSensor<CCI_PositioningSensor>("positioning");
   }
   catch(CARGoSException&) {
      m_pcPositioning = nullptr;
   }

   try {
      m_pcLidar = GetSensor<CCI_PhotorealisticLidarSensor>("photorealistic_lidar");
   }
   catch(CARGoSException&) {
      m_pcLidar = nullptr;
   }

   try {
      m_pcCamera = GetSensor<CCI_PhotorealisticCameraSensor>("photorealistic_camera");
   }
   catch(CARGoSException&) {
      m_pcCamera = nullptr;
   }

   try {
      m_pcWheelEncoders = GetSensor<CCI_DifferentialSteeringSensor>("differential_steering");
   }
   catch(CARGoSException&) {
      m_pcWheelEncoders = nullptr;
   }

   try {
      m_pcIMU = GetSensor<CCI_IMUSensor>("imu");
   }
   catch(CARGoSException&) {
      m_pcIMU = nullptr;
   }

   try {
      m_pcOdometry = GetSensor<CCI_OdometrySensor>("odometry");
   }
   catch(CARGoSException&) {
      m_pcOdometry = nullptr;
   }

   /* Parameters */
   GetNodeAttributeOrDefault(t_tree, "robot_id", m_strRobotId, m_strRobotId);
   GetNodeAttributeOrDefault(t_tree, "track_gauge", m_fTrackGauge, m_fTrackGauge);
   GetNodeAttributeOrDefault(t_tree, "max_speed", m_fMaxSpeed, m_fMaxSpeed);
}

/****************************************/
/****************************************/

void CSwarmdeckRobotController::ControlStep() {
   if(m_pcWheels == nullptr) return;

   /* Convert linear (m/s) & angular (rad/s) velocity to left & right wheel speeds (cm/s) */
   Real fLeft_m_s = m_fTargetLinearVel - (m_fTargetAngularVel * m_fTrackGauge / 2.0);
   Real fRight_m_s = m_fTargetLinearVel + (m_fTargetAngularVel * m_fTrackGauge / 2.0);

   Real fLeft_cm_s = fLeft_m_s * 100.0;
   Real fRight_cm_s = fRight_m_s * 100.0;

   /* Clamp to max speed */
   fLeft_cm_s = std::max(-m_fMaxSpeed, std::min(m_fMaxSpeed, fLeft_cm_s));
   fRight_cm_s = std::max(-m_fMaxSpeed, std::min(m_fMaxSpeed, fRight_cm_s));

   m_fLastLeftSpeed = fLeft_cm_s;
   m_fLastRightSpeed = fRight_cm_s;

   m_pcWheels->SetLinearVelocity(fLeft_cm_s, fRight_cm_s);
}

/****************************************/
/****************************************/

void CSwarmdeckRobotController::SetVelocity(Real f_linear_m_s, Real f_angular_rad_s) {
   m_fTargetLinearVel = f_linear_m_s;
   m_fTargetAngularVel = f_angular_rad_s;
}

/****************************************/
/****************************************/

void CSwarmdeckRobotController::Reset() {
   m_fTargetLinearVel = 0.0;
   m_fTargetAngularVel = 0.0;
   m_fLastLeftSpeed = 0.0;
   m_fLastRightSpeed = 0.0;
   if(m_pcWheels != nullptr) {
      m_pcWheels->SetLinearVelocity(0.0, 0.0);
   }
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CSwarmdeckRobotController, "swarmdeck_robot_controller");
