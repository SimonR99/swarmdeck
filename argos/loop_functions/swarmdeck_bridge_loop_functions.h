/**
 * @file swarmdeck_bridge_loop_functions.h
 *
 * The whole ROS boundary of the ARGoS simulation. Every tick (or every
 * `exchange_period` ticks) this serialises each robot's ground truth,
 * odometry, wheel encoders, IMU, lidar scan and camera frame onto a Unix
 * domain socket, and applies the cmd_vel, teleport and reset commands that
 * come back.
 *
 * It is a loop function rather than a medium because a bridge is exactly what
 * the single <loop_functions> slot is for. The Ultra-Fusion link is the other
 * way round: ARGoS accepts one loop function and a list of media, so the
 * external estimator lives in <media> and the two never contend.
 *
 * ARGoS links no ROS. Everything on the far side of the socket is ROS 2, in
 * its own container.
 */

#ifndef SWARMDECK_BRIDGE_LOOP_FUNCTIONS_H
#define SWARMDECK_BRIDGE_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/math/vector3.h>

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

class CSwarmdeckRobotController;

namespace argos {
   class CComposableEntity;
   class CEmbodiedEntity;
}

using namespace argos;

class CSwarmdeckBridgeLoopFunctions : public CLoopFunctions {

public:

   CSwarmdeckBridgeLoopFunctions();
   virtual ~CSwarmdeckBridgeLoopFunctions() {}

   virtual void Init(TConfigurationNode& t_tree) override;
   virtual void PostStep() override;
   virtual void Reset() override;
   virtual void Destroy() override;

private:

   void Connect();
   void SendAll(const void* pt_data, size_t un_size);
   void RecvAll(void* pt_data, size_t un_size);
   void Append(const void* pt_data, size_t un_size);
   bool HasDataToRead();

   void HandleCommands(UInt32 un_tick);
   void TeleportRobot(size_t un_index, const CVector3& c_pos, const CQuaternion& c_quat);
   void ResetAllRobots();
   void PaceRealTime();

   int m_nSocket = -1;
   std::string m_strSocketPath = "/run/swarmdeck/argos.sock";
   Real m_fTimeout = 60.0;
   Real m_fConnectTimeout = 120.0;
   UInt32 m_unTicksPerSecond = 100;
   UInt32 m_unExchangePeriod = 10;
   bool m_bRealtimeStreaming = true;

   /* Wall-clock seconds of simulation per second of real time. 1 holds the
    * simulation to real time, which is what the operator UI, the WebRTC video
    * and teleoperation all assume; 0 runs as fast as the estimator allows.
    * Headless ARGoS has no pacing of its own (only the filament viewer's
    * `speed` attribute does this), and an unpaced run either sprints away
    * from the video or crawls behind it. */
   Real m_fRealtimeFactor = 1.0;
   std::chrono::steady_clock::time_point m_tStart;
   bool m_bStarted = false;

   std::vector<std::string> m_vecRobotIds;
   std::vector<CSwarmdeckRobotController*> m_vecControllers;
   std::vector<CEmbodiedEntity*> m_vecEmbodiedEntities;
   std::vector<CVector3> m_vecInitialPositions;
   std::vector<CQuaternion> m_vecInitialOrientations;

   /* Previous exchange's ground-truth pose, for the finite-differenced twist.
    * ARGoS embodied entities expose a pose and no velocity: the velocity
    * lives in whichever physics model owns the body, behind no common
    * interface. Differencing the pose over the exchange interval is both the
    * portable answer and an honest one, since it is what a perfect
    * ground-truth odometry would report. */
   std::vector<CVector3> m_vecPrevPositions;
   std::vector<CQuaternion> m_vecPrevOrientations;
   bool m_bHavePrevPose = false;
   UInt32 m_unPrevTick = 0;

   std::vector<UInt8> m_vecSendBuffer;
};

#endif
