/**
 * @file swarmdeck_bridge_loop_functions.cpp
 *
 * Wire format, little-endian throughout, no padding (fields are appended
 * one at a time, so the C++ side never inserts any and the Python side
 * unpacks with '<' formats):
 *
 *   observation := "SDB2" tick:u32 ticks_per_second:u32 robot_count:u32
 *                  robot*
 *   robot       := id_len:u8 id:bytes
 *                  gt_pos:3f64 gt_quat:4f64(wxyz) gt_lin:3f64 gt_ang:3f64
 *                  has_odometry:u8 [ valid:u8 pos:3f64 quat:4f64
 *                                    lin:3f64 ang:3f64 tick:u32 ]
 *                  has_encoders:u8 [ vel:2f64 dist:2f64 ]
 *                  has_imu:u8      [ gyro:3f64 accel:3f64 ]
 *                  has_lidar:u8    [ tick:u32 rings:u32 azimuths:u32
 *                                    max_range:f32 count:u32
 *                                    (range:f32 x:f32 y:f32 z:f32
 *                                     ring:u16 hit:u8)* ]
 *                  has_camera:u8   [ tick:u32 w:u32 h:u32 fov_deg:f32
 *                                    rgb:w*h*3 has_depth:u8 [ depth:w*h*4 ] ]
 *
 *   command     := "SDCMD" tick:u32 robot_count:u32 cmd* world_reset:u8
 *   cmd         := id_len:u8 id:bytes linear_x:f32 angular_z:f32
 *                  teleport:u8 [ pos:3f64 quat:4f64 ]
 *
 * The magic is "SDB2" and not "SDBR": the odometry block was added after the
 * first version, and a length-prefixed stream that is misparsed by one byte
 * does not fail, it silently reports garbage poses. A changed magic makes a
 * version mismatch loud on the first packet.
 */

#include "swarmdeck_bridge_loop_functions.h"
#include "swarmdeck_robot_controller.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/core/simulator/physics_engine/physics_engine.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_imu_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_odometry_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_photorealistic_lidar_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_photorealistic_camera_sensor.h>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <poll.h>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <iostream>
#include <sstream>
#include <thread>

static const UInt8 kObservationMagic[4] = {'S', 'D', 'B', '2'};
static const UInt8 kCommandMagic[5]     = {'S', 'D', 'C', 'M', 'D'};

CSwarmdeckBridgeLoopFunctions::CSwarmdeckBridgeLoopFunctions() {
}

void CSwarmdeckBridgeLoopFunctions::Init(TConfigurationNode& t_tree) {
   GetNodeAttributeOrDefault(t_tree, "socket", m_strSocketPath, m_strSocketPath);
   GetNodeAttributeOrDefault(t_tree, "timeout", m_fTimeout, m_fTimeout);
   GetNodeAttributeOrDefault(t_tree, "connect_timeout", m_fConnectTimeout, m_fConnectTimeout);
   GetNodeAttributeOrDefault(t_tree, "exchange_period", m_unExchangePeriod, m_unExchangePeriod);
   GetNodeAttributeOrDefault(t_tree, "realtime", m_bRealtimeStreaming, m_bRealtimeStreaming);
   GetNodeAttributeOrDefault(t_tree, "realtime_factor", m_fRealtimeFactor, m_fRealtimeFactor);

   if(m_unExchangePeriod == 0) {
      THROW_ARGOSEXCEPTION("[swarmdeck_bridge] exchange_period must be >= 1");
   }

   /* The tick rate is a property of the simulation, not of any one engine:
    * CPhysicsEngine holds it statically. Reading it off an engine looked up
    * by the name "jolt" broke the moment a run used a different engine. */
   Real fInverseTick = CPhysicsEngine::GetInverseSimulationClockTick();
   m_unTicksPerSecond = (fInverseTick > 0.0)
      ? static_cast<UInt32>(std::lround(fInverseTick))
      : 100;

   std::string strRobots;
   GetNodeAttributeOrDefault(t_tree, "robots", strRobots, strRobots);
   m_vecRobotIds.clear();
   if(!strRobots.empty()) {
      std::stringstream ss(strRobots);
      std::string strItem;
      while(std::getline(ss, strItem, ',')) {
         if(!strItem.empty()) {
            m_vecRobotIds.push_back(strItem);
         }
      }
   }
   if(m_vecRobotIds.empty()) {
      THROW_ARGOSEXCEPTION("[swarmdeck_bridge] no robots listed; set robots=\"id,id,...\"");
   }

   m_vecControllers.clear();
   m_vecEmbodiedEntities.clear();
   m_vecInitialPositions.clear();
   m_vecInitialOrientations.clear();

   for(const std::string& strId : m_vecRobotIds) {
      try {
         CEntity& cEntity = GetSpace().GetEntity(strId);
         CComposableEntity* pcComposable = dynamic_cast<CComposableEntity*>(&cEntity);
         if(!pcComposable) {
            THROW_ARGOSEXCEPTION("Entity \"" << strId << "\" is not composable.");
         }

         CControllableEntity& cControllable = pcComposable->GetComponent<CControllableEntity>("controller");
         CSwarmdeckRobotController* pcCtrl = dynamic_cast<CSwarmdeckRobotController*>(&cControllable.GetController());
         if(!pcCtrl) {
            THROW_ARGOSEXCEPTION("Entity \"" << strId << "\" does not have a CSwarmdeckRobotController.");
         }

         CEmbodiedEntity& cEmbodied = pcComposable->GetComponent<CEmbodiedEntity>("body");

         m_vecControllers.push_back(pcCtrl);
         m_vecEmbodiedEntities.push_back(&cEmbodied);
         m_vecInitialPositions.push_back(cEmbodied.GetOriginAnchor().Position);
         m_vecInitialOrientations.push_back(cEmbodied.GetOriginAnchor().Orientation);
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error locating robot \"" << strId << "\".", ex);
      }
   }

   m_vecPrevPositions = m_vecInitialPositions;
   m_vecPrevOrientations = m_vecInitialOrientations;
   m_bHavePrevPose = false;
   m_unPrevTick = 0;

   Connect();
}

void CSwarmdeckBridgeLoopFunctions::Connect() {
   if(m_nSocket >= 0) {
      ::close(m_nSocket);
      m_nSocket = -1;
   }

   m_nSocket = ::socket(AF_UNIX, SOCK_STREAM, 0);
   if(m_nSocket < 0) {
      THROW_ARGOSEXCEPTION("[swarmdeck_bridge] Failed to create Unix socket: "
                           << ::strerror(errno));
   }

   struct sockaddr_un sAddr;
   ::memset(&sAddr, 0, sizeof(sAddr));
   sAddr.sun_family = AF_UNIX;
   ::strncpy(sAddr.sun_path, m_strSocketPath.c_str(), sizeof(sAddr.sun_path) - 1);

   std::cout << "[swarmdeck_bridge] Connecting to " << m_strSocketPath
             << " (timeout " << m_fConnectTimeout << "s, realtime_factor "
             << m_fRealtimeFactor << ")..." << std::endl;

   UInt32 unElapsedMs = 0;
   UInt32 unTimeoutMs = static_cast<UInt32>(m_fConnectTimeout * 1000.0);
   bool bConnected = false;

   while(unElapsedMs < unTimeoutMs) {
      if(::connect(m_nSocket, reinterpret_cast<struct sockaddr*>(&sAddr), sizeof(sAddr)) == 0) {
         bConnected = true;
         break;
      }
      ::usleep(100000);
      unElapsedMs += 100;
   }

   if(!bConnected) {
      ::close(m_nSocket);
      m_nSocket = -1;
      THROW_ARGOSEXCEPTION("[swarmdeck_bridge] Connection timed out for " << m_strSocketPath);
   }

   std::cout << "[swarmdeck_bridge] Connected to the SwarmDeck ROS 2 bridge." << std::endl;
}

void CSwarmdeckBridgeLoopFunctions::Append(const void* pt_data, size_t un_size) {
   const UInt8* punBytes = reinterpret_cast<const UInt8*>(pt_data);
   m_vecSendBuffer.insert(m_vecSendBuffer.end(), punBytes, punBytes + un_size);
}

void CSwarmdeckBridgeLoopFunctions::SendAll(const void* pt_data, size_t un_size) {
   if(m_nSocket < 0) return;
   const char* pchBuffer = reinterpret_cast<const char*>(pt_data);
   size_t unWritten = 0;
   while(unWritten < un_size) {
      ssize_t nSent = ::send(m_nSocket, pchBuffer + unWritten, un_size - unWritten, MSG_NOSIGNAL);
      if(nSent < 0) {
         if(errno == EINTR) continue;
         THROW_ARGOSEXCEPTION("[swarmdeck_bridge] Socket send error: " << ::strerror(errno));
      }
      unWritten += size_t(nSent);
   }
}

void CSwarmdeckBridgeLoopFunctions::RecvAll(void* pt_data, size_t un_size) {
   if(m_nSocket < 0) return;
   char* pchBuffer = reinterpret_cast<char*>(pt_data);
   size_t unRead = 0;
   while(unRead < un_size) {
      ssize_t nGot = ::recv(m_nSocket, pchBuffer + unRead, un_size - unRead, 0);
      if(nGot == 0) {
         THROW_ARGOSEXCEPTION("[swarmdeck_bridge] Bridge closed the connection");
      }
      if(nGot < 0) {
         if(errno == EINTR) continue;
         if(errno == EAGAIN || errno == EWOULDBLOCK) {
            THROW_ARGOSEXCEPTION("[swarmdeck_bridge] Socket recv timed out ("
                                 << m_fTimeout << "s)");
         }
         THROW_ARGOSEXCEPTION("[swarmdeck_bridge] Socket recv error: " << ::strerror(errno));
      }
      unRead += size_t(nGot);
   }
}

bool CSwarmdeckBridgeLoopFunctions::HasDataToRead() {
   if(m_nSocket < 0) return false;
   struct pollfd sPfd;
   sPfd.fd = m_nSocket;
   sPfd.events = POLLIN;
   sPfd.revents = 0;
   int nRet = ::poll(&sPfd, 1, 0);
   return (nRet > 0 && (sPfd.revents & POLLIN));
}

/**
 * Hold the simulation to wall-clock time.
 *
 * The operator dashboard, the WebRTC video and teleoperation all run on real
 * time while everything inside ROS runs on the bridged /clock. Letting the
 * simulation free-run decouples the two in whichever direction the machine
 * happens to favour: faster than real time and the map races ahead of the
 * video, slower and a teleop command lands seconds after it was pressed.
 *
 * This only ever sleeps. When the simulation is already behind (which is the
 * normal state with a real estimator in the lockstep loop) it returns
 * immediately rather than trying to catch up, because catching up would mean
 * skipping the exchange that carries the sensor data.
 */
void CSwarmdeckBridgeLoopFunctions::PaceRealTime() {
   if(m_fRealtimeFactor <= 0.0) return;
   if(!m_bStarted) {
      m_tStart = std::chrono::steady_clock::now();
      m_bStarted = true;
      return;
   }
   Real fSimSeconds = Real(GetSpace().GetSimulationClock()) / Real(m_unTicksPerSecond);
   Real fTargetSeconds = fSimSeconds / m_fRealtimeFactor;
   auto tTarget = m_tStart + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<Real>(fTargetSeconds));
   auto tNow = std::chrono::steady_clock::now();
   if(tTarget > tNow) {
      std::this_thread::sleep_for(tTarget - tNow);
   }
}

void CSwarmdeckBridgeLoopFunctions::PostStep() {
   UInt32 unTick = GetSpace().GetSimulationClock();

   PaceRealTime();

   if(unTick % m_unExchangePeriod != 0) {
      return;
   }

   /* Seconds since the previous exchange, for the finite-differenced twist. */
   Real fDt = m_bHavePrevPose
      ? Real(unTick - m_unPrevTick) / Real(m_unTicksPerSecond)
      : 0.0;

   m_vecSendBuffer.clear();

   Append(kObservationMagic, sizeof(kObservationMagic));
   Append(&unTick, sizeof(unTick));
   Append(&m_unTicksPerSecond, sizeof(m_unTicksPerSecond));
   UInt32 unRobotCount = static_cast<UInt32>(m_vecRobotIds.size());
   Append(&unRobotCount, sizeof(unRobotCount));

   for(size_t i = 0; i < m_vecRobotIds.size(); ++i) {
      const std::string& strId = m_vecRobotIds[i];
      CSwarmdeckRobotController* pcCtrl = m_vecControllers[i];
      CEmbodiedEntity* pcEmbodied = m_vecEmbodiedEntities[i];

      UInt8 unIdLen = static_cast<UInt8>(strId.size());
      Append(&unIdLen, sizeof(unIdLen));
      Append(strId.data(), unIdLen);

      /* --- ground truth ------------------------------------------------- */
      const CVector3& cPos = pcEmbodied->GetOriginAnchor().Position;
      const CQuaternion& cQuat = pcEmbodied->GetOriginAnchor().Orientation;
      double fPos[3] = {cPos.GetX(), cPos.GetY(), cPos.GetZ()};
      double fQuat[4] = {cQuat.GetW(), cQuat.GetX(), cQuat.GetY(), cQuat.GetZ()};
      Append(fPos, sizeof(fPos));
      Append(fQuat, sizeof(fQuat));

      double fLinVel[3] = {0, 0, 0};
      double fAngVel[3] = {0, 0, 0};
      if(fDt > 0.0) {
         /* Body-frame twist, matching what nav_msgs/Odometry expects in
          * twist.twist: the world-frame delta rotated back into base_link. */
         CVector3 cDelta = (cPos - m_vecPrevPositions[i]) / fDt;
         CQuaternion cInv = cQuat.Inverse();
         cDelta.Rotate(cInv);
         fLinVel[0] = cDelta.GetX();
         fLinVel[1] = cDelta.GetY();
         fLinVel[2] = cDelta.GetZ();

         CQuaternion cDeltaQ = m_vecPrevOrientations[i].Inverse() * cQuat;
         CVector3 cAxis;
         CRadians cAngle;
         cDeltaQ.ToAngleAxis(cAngle, cAxis);
         /* ToAngleAxis returns [0, 2pi); fold the far half onto a signed
          * rotation so a small clockwise turn does not read as a large
          * counter-clockwise one. */
         Real fAngle = cAngle.GetValue();
         if(fAngle > ARGOS_PI) fAngle -= 2.0 * ARGOS_PI;
         cAxis *= fAngle / fDt;
         fAngVel[0] = cAxis.GetX();
         fAngVel[1] = cAxis.GetY();
         fAngVel[2] = cAxis.GetZ();
      }
      Append(fLinVel, sizeof(fLinVel));
      Append(fAngVel, sizeof(fAngVel));

      m_vecPrevPositions[i] = cPos;
      m_vecPrevOrientations[i] = cQuat;

      /* --- odometry ------------------------------------------------------
       * This is the pose the whole stack navigates and maps on, and it is
       * deliberately NOT the ground truth above. With
       * <odometry implementation="external"> it is whatever Ultra-Fusion
       * actually estimated from the simulated IMU, lidar and wheels, so it
       * drifts the way a real front-end drifts. Publishing ground truth here
       * would make SwarmDeck's collaborative pose-graph merge trivially
       * correct and prove nothing. `Valid` is false until the estimator has
       * converged; the ROS side must not publish a pose before then. */
      CCI_OdometrySensor* pcOdometry = pcCtrl->GetOdometry();
      UInt8 unHasOdometry = (pcOdometry != nullptr) ? 1 : 0;
      Append(&unHasOdometry, sizeof(unHasOdometry));
      if(unHasOdometry) {
         const CCI_OdometrySensor::SReading& sOdom = pcOdometry->GetReading();
         UInt8 unValid = sOdom.Valid ? 1 : 0;
         double fOdomPos[3] = {sOdom.Position.GetX(),
                               sOdom.Position.GetY(),
                               sOdom.Position.GetZ()};
         double fOdomQuat[4] = {sOdom.Orientation.GetW(),
                                sOdom.Orientation.GetX(),
                                sOdom.Orientation.GetY(),
                                sOdom.Orientation.GetZ()};
         double fOdomLin[3] = {sOdom.LinearVelocity.GetX(),
                               sOdom.LinearVelocity.GetY(),
                               sOdom.LinearVelocity.GetZ()};
         double fOdomAng[3] = {sOdom.AngularVelocity.GetX(),
                               sOdom.AngularVelocity.GetY(),
                               sOdom.AngularVelocity.GetZ()};
         UInt32 unOdomTick = sOdom.Tick;
         Append(&unValid, sizeof(unValid));
         Append(fOdomPos, sizeof(fOdomPos));
         Append(fOdomQuat, sizeof(fOdomQuat));
         Append(fOdomLin, sizeof(fOdomLin));
         Append(fOdomAng, sizeof(fOdomAng));
         Append(&unOdomTick, sizeof(unOdomTick));
      }

      /* --- wheel encoders ------------------------------------------------ */
      CCI_DifferentialSteeringSensor* pcEncoders = pcCtrl->GetWheelEncoders();
      UInt8 unHasEncoders = (pcEncoders != nullptr) ? 1 : 0;
      Append(&unHasEncoders, sizeof(unHasEncoders));
      if(unHasEncoders) {
         /* ARGoS reports wheel motion in cm and cm/s; ROS wants metres. */
         double fWheelVel[2] = {
            pcEncoders->GetReading().VelocityLeftWheel / 100.0,
            pcEncoders->GetReading().VelocityRightWheel / 100.0
         };
         double fWheelDist[2] = {
            pcEncoders->GetReading().CoveredDistanceLeftWheel / 100.0,
            pcEncoders->GetReading().CoveredDistanceRightWheel / 100.0
         };
         Append(fWheelVel, sizeof(fWheelVel));
         Append(fWheelDist, sizeof(fWheelDist));
      }

      /* --- IMU ------------------------------------------------------------ */
      CCI_IMUSensor* pcIMU = pcCtrl->GetIMU();
      UInt8 unHasIMU = (pcIMU != nullptr) ? 1 : 0;
      Append(&unHasIMU, sizeof(unHasIMU));
      if(unHasIMU) {
         const CCI_IMUSensor::SReading& sReading = pcIMU->GetReading();
         double fGyro[3] = {
            sReading.AngularVelocity.GetX(),
            sReading.AngularVelocity.GetY(),
            sReading.AngularVelocity.GetZ()
         };
         double fAccel[3] = {
            sReading.LinearAcceleration.GetX(),
            sReading.LinearAcceleration.GetY(),
            sReading.LinearAcceleration.GetZ()
         };
         Append(fGyro, sizeof(fGyro));
         Append(fAccel, sizeof(fAccel));
      }

      /* --- lidar ---------------------------------------------------------- */
      CCI_PhotorealisticLidarSensor* pcLidar = pcCtrl->GetLidar();
      UInt8 unHasLidar = (pcLidar != nullptr) ? 1 : 0;
      Append(&unHasLidar, sizeof(unHasLidar));
      if(unHasLidar) {
         const CCI_PhotorealisticLidarSensor::SScan& sScan = pcLidar->GetScan();
         /* The scan's own tick, not the current one: frames are pipelined, so
          * a scan seen now normally depicts the previous tick. The ROS side
          * stamps from this. */
         UInt32 unScanTick = sScan.Tick;
         UInt32 unRings = sScan.NumRings;
         UInt32 unAzimuths = sScan.NumAzimuths;
         float fMaxRange = float(sScan.MaxRange);
         UInt32 unNumReadings = static_cast<UInt32>(sScan.Readings.size());

         Append(&unScanTick, sizeof(unScanTick));
         Append(&unRings, sizeof(unRings));
         Append(&unAzimuths, sizeof(unAzimuths));
         Append(&fMaxRange, sizeof(fMaxRange));
         Append(&unNumReadings, sizeof(unNumReadings));

         for(const auto& sReading : sScan.Readings) {
            float fRange = float(sReading.Range);
            float fX = float(sReading.Position.GetX());
            float fY = float(sReading.Position.GetY());
            float fZ = float(sReading.Position.GetZ());
            UInt16 unRing = static_cast<UInt16>(sReading.Ring);
            UInt8 unHit = sReading.Hit ? 1 : 0;

            Append(&fRange, sizeof(fRange));
            Append(&fX, sizeof(fX));
            Append(&fY, sizeof(fY));
            Append(&fZ, sizeof(fZ));
            Append(&unRing, sizeof(unRing));
            Append(&unHit, sizeof(unHit));
         }
      }

      /* --- camera ---------------------------------------------------------- */
      CCI_PhotorealisticCameraSensor* pcCamera = pcCtrl->GetCamera();
      UInt8 unHasCamera = (pcCamera != nullptr) ? 1 : 0;
      Append(&unHasCamera, sizeof(unHasCamera));
      if(unHasCamera) {
         const CCI_PhotorealisticCameraSensor::SFrame& sFrame = pcCamera->GetFrame();
         UInt32 unCamTick = sFrame.Tick;
         UInt32 unWidth = sFrame.Width;
         UInt32 unHeight = sFrame.Height;
         float fFOV = float(sFrame.FieldOfView);

         Append(&unCamTick, sizeof(unCamTick));
         Append(&unWidth, sizeof(unWidth));
         Append(&unHeight, sizeof(unHeight));
         Append(&fFOV, sizeof(fFOV));

         const size_t unRGBBytes = size_t(unWidth) * size_t(unHeight) * 3;
         if(sFrame.RGB.size() == unRGBBytes) {
            Append(sFrame.RGB.data(), unRGBBytes);
         }
         else {
            /* The first frames of a pipelined camera are empty. Send black
             * rather than a short block: the stream is not self-delimiting,
             * so a missing payload desynchronises everything after it. */
            std::vector<UInt8> vecZeros(unRGBBytes, 0);
            Append(vecZeros.data(), vecZeros.size());
         }

         UInt8 unHasDepth = (!sFrame.Depth.empty()) ? 1 : 0;
         Append(&unHasDepth, sizeof(unHasDepth));
         if(unHasDepth) {
            std::vector<float> vecFloatDepth(sFrame.Depth.size());
            for(size_t p = 0; p < sFrame.Depth.size(); ++p) {
               vecFloatDepth[p] = static_cast<float>(sFrame.Depth[p]);
            }
            Append(vecFloatDepth.data(), vecFloatDepth.size() * sizeof(float));
         }
      }
   }

   m_bHavePrevPose = true;
   m_unPrevTick = unTick;

   SendAll(m_vecSendBuffer.data(), m_vecSendBuffer.size());

   /* In realtime streaming mode the far side answers when it has something to
    * say; in lockstep it always answers, and the simulation waits. */
   if(m_bRealtimeStreaming) {
      while(HasDataToRead()) {
         HandleCommands(unTick);
      }
   }
   else {
      HandleCommands(unTick);
   }
}

void CSwarmdeckBridgeLoopFunctions::HandleCommands(UInt32) {
   UInt8 unMagic[5] = {0};
   RecvAll(unMagic, sizeof(unMagic));
   if(::memcmp(unMagic, kCommandMagic, sizeof(kCommandMagic)) != 0) {
      THROW_ARGOSEXCEPTION("[swarmdeck_bridge] Invalid command header magic");
   }

   UInt32 unCmdTick = 0;
   UInt32 unRobotCount = 0;
   RecvAll(&unCmdTick, sizeof(unCmdTick));
   RecvAll(&unRobotCount, sizeof(unRobotCount));

   for(UInt32 i = 0; i < unRobotCount; ++i) {
      UInt8 unIdLen = 0;
      RecvAll(&unIdLen, sizeof(unIdLen));
      std::string strId(unIdLen, '\0');
      if(unIdLen > 0) {
         RecvAll(&strId[0], unIdLen);
      }

      float fLinearX = 0.0f;
      float fAngularZ = 0.0f;
      RecvAll(&fLinearX, sizeof(fLinearX));
      RecvAll(&fAngularZ, sizeof(fAngularZ));

      UInt8 unTeleportFlag = 0;
      RecvAll(&unTeleportFlag, sizeof(unTeleportFlag));
      double fTeleportPos[3] = {0, 0, 0};
      double fTeleportQuat[4] = {1, 0, 0, 0};
      if(unTeleportFlag) {
         RecvAll(fTeleportPos, sizeof(fTeleportPos));
         RecvAll(fTeleportQuat, sizeof(fTeleportQuat));
      }

      for(size_t r = 0; r < m_vecRobotIds.size(); ++r) {
         if(m_vecRobotIds[r] == strId) {
            m_vecControllers[r]->SetVelocity(fLinearX, fAngularZ);
            if(unTeleportFlag) {
               CVector3 cPos(fTeleportPos[0], fTeleportPos[1], fTeleportPos[2]);
               CQuaternion cQuat(fTeleportQuat[0], fTeleportQuat[1],
                                 fTeleportQuat[2], fTeleportQuat[3]);
               TeleportRobot(r, cPos, cQuat);
            }
            break;
         }
      }
   }

   UInt8 unWorldResetFlag = 0;
   RecvAll(&unWorldResetFlag, sizeof(unWorldResetFlag));
   if(unWorldResetFlag) {
      ResetAllRobots();
   }
}

void CSwarmdeckBridgeLoopFunctions::TeleportRobot(size_t un_index,
                                                  const CVector3& c_pos,
                                                  const CQuaternion& c_quat) {
   if(un_index >= m_vecEmbodiedEntities.size()) return;
   CEmbodiedEntity* pcEmbodied = m_vecEmbodiedEntities[un_index];
   pcEmbodied->MoveTo(c_pos, c_quat);
   m_vecControllers[un_index]->Reset();
   /* Drop the pose history with the robot: differencing across a teleport
    * would report a jump of tens of metres per second as ground-truth
    * velocity, and the operator's reset is exactly when that happens. */
   m_vecPrevPositions[un_index] = c_pos;
   m_vecPrevOrientations[un_index] = c_quat;
}

void CSwarmdeckBridgeLoopFunctions::ResetAllRobots() {
   for(size_t i = 0; i < m_vecEmbodiedEntities.size(); ++i) {
      TeleportRobot(i, m_vecInitialPositions[i], m_vecInitialOrientations[i]);
   }
}

void CSwarmdeckBridgeLoopFunctions::Reset() {
   ResetAllRobots();
   m_bHavePrevPose = false;
   m_bStarted = false;
}

void CSwarmdeckBridgeLoopFunctions::Destroy() {
   if(m_nSocket >= 0) {
      ::close(m_nSocket);
      m_nSocket = -1;
   }
}

REGISTER_LOOP_FUNCTIONS(CSwarmdeckBridgeLoopFunctions, "swarmdeck_bridge");
