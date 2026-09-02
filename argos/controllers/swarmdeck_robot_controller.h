/**
 * @file swarmdeck_robot_controller.h
 *
 * ARGoS controller for SwarmDeck multi-robot fleets.
 * Receives cmd_vel velocity commands from the bridge loop function and drives
 * the differential/tracked base while making sensor streams available.
 */

#ifndef SWARMDECK_ROBOT_CONTROLLER_H
#define SWARMDECK_ROBOT_CONTROLLER_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/utility/math/angles.h>
#include <argos3/core/utility/math/vector3.h>

#include <string>

namespace argos {
   class CCI_DifferentialSteeringActuator;
   class CCI_DifferentialSteeringSensor;
   class CCI_PositioningSensor;
   class CCI_PhotorealisticLidarSensor;
   class CCI_PhotorealisticCameraSensor;
   class CCI_IMUSensor;
   class CCI_OdometrySensor;
}

using namespace argos;

class CSwarmdeckRobotController : public CCI_Controller {

public:

   CSwarmdeckRobotController();
   virtual ~CSwarmdeckRobotController() {}

   virtual void Init(TConfigurationNode& t_tree) override;
   virtual void ControlStep() override;
   virtual void Reset() override;
   virtual void Destroy() override {}

   /** Set target velocity from cmd_vel */
   void SetVelocity(Real f_linear_m_s, Real f_angular_rad_s);

   /** Get robot namespace / ID */
   const std::string& GetRobotId() const { return m_strRobotId; }

   /** Track gauge in meters */
   Real GetTrackGauge() const { return m_fTrackGauge; }

   /** Get commanded wheel velocities in cm/s */
   void GetWheelCommands(Real& f_left, Real& f_right) const {
      f_left = m_fLastLeftSpeed;
      f_right = m_fLastRightSpeed;
   }

   /** Sensor accessors */
   CCI_DifferentialSteeringActuator* GetWheelsActuator() { return m_pcWheels; }
   CCI_DifferentialSteeringSensor*   GetWheelEncoders()  { return m_pcWheelEncoders; }
   CCI_PositioningSensor*            GetPositioning()    { return m_pcPositioning; }
   CCI_PhotorealisticLidarSensor*    GetLidar()          { return m_pcLidar; }
   CCI_PhotorealisticCameraSensor*   GetCamera()         { return m_pcCamera; }
   CCI_IMUSensor*                    GetIMU()            { return m_pcIMU; }
   CCI_OdometrySensor*               GetOdometry()       { return m_pcOdometry; }

private:

   CCI_DifferentialSteeringActuator* m_pcWheels = nullptr;
   CCI_DifferentialSteeringSensor*   m_pcWheelEncoders = nullptr;
   CCI_PositioningSensor*            m_pcPositioning = nullptr;
   CCI_PhotorealisticLidarSensor*    m_pcLidar = nullptr;
   CCI_PhotorealisticCameraSensor*   m_pcCamera = nullptr;
   CCI_IMUSensor*                    m_pcIMU = nullptr;
   CCI_OdometrySensor*               m_pcOdometry = nullptr;

   std::string m_strRobotId = "robot_0";
   Real m_fTrackGauge = 0.58;       // Track gauge / wheel separation in metres
   Real m_fMaxSpeed = 150.0;        // Max wheel linear speed in cm/s (1.5 m/s)

   Real m_fTargetLinearVel = 0.0;   // m/s
   Real m_fTargetAngularVel = 0.0;  // rad/s

   Real m_fLastLeftSpeed = 0.0;     // cm/s
   Real m_fLastRightSpeed = 0.0;    // cm/s

};

#endif
