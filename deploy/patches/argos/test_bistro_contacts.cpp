// Reproduce the invisible manhole contact using triangles from the real asset.
#include "swarmdeck_step.h"
#include "test_layers.h"
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <fstream>
#include <iostream>
#include <string>

int traverse(const char* path, bool corrected, bool wall) {
  Layers layers;
  Pair pair;
  BroadPair broad;
  PhysicsSystem system;
  system.Init(64, 0, 256, 256, layers, broad, pair);
  system.SetGravity(Vec3(0, 0, -9.81f));
  auto& bodies = system.GetBodyInterface();

  std::ifstream input(path, std::ios::binary);
  TriangleList triangles, flipped;
  float p[9];
  while (input.read(reinterpret_cast<char*>(p), sizeof(p))) {
    Float3 a(p[0], p[1], p[2]), b(p[3], p[4], p[5]), c(p[6], p[7], p[8]);
    triangles.emplace_back(a, b, c);
    flipped.emplace_back(a, c, b);
  }
  if (triangles.empty()) return 2;
  // Match ARGoS's double-sided collider: two independently cooked windings.
  StaticCompoundShapeSettings road;
  road.AddShape(Vec3::sZero(), Quat::sIdentity(), MeshShapeSettings(triangles).Create().Get());
  road.AddShape(Vec3::sZero(), Quat::sIdentity(), MeshShapeSettings(flipped).Create().Get());
  bodies.CreateAndAddBody(
      BodyCreationSettings(road.Create().Get(), RVec3::sZero(), Quat::sIdentity(),
                           EMotionType::Static, 0), EActivation::DontActivate);

  if (wall) {
    bodies.CreateAndAddBody(
        BodyCreationSettings(new BoxShape(Vec3(2, .1f, 1)), RVec3(-12, 1, 1),
                             Quat::sIdentity(), EMotionType::Static, 0),
        EActivation::DontActivate);
  }

  // Actual R0 pose and Bunker dimensions, heading through Manhole3.
  Quat rotation = Quat::sRotation(Vec3::sAxisZ(), -1.8335432f);
  RVec3 position(-12.18042f, 2.75487f, .022705f + .19f);
  BodyCreationSettings robot(new BoxShape(Vec3(1.023f / 2, .778f / 2, .19f)),
                             position, rotation, EMotionType::Dynamic, 1);
  robot.mEnhancedInternalEdgeRemoval = corrected;
  robot.mFriction = 0;
  robot.mLinearDamping = 0;
  robot.mAngularDamping = 0;
  robot.mAllowSleeping = false;
  robot.mAllowedDOFs = EAllowedDOFs::TranslationX | EAllowedDOFs::TranslationY |
                      EAllowedDOFs::TranslationZ | EAllowedDOFs::RotationZ;
  robot.mMotionQuality = EMotionQuality::LinearCast;
  auto id = bodies.CreateAndAddBody(robot, EActivation::Activate);
  system.OptimizeBroadPhase();
  TempAllocatorImpl allocator(10 * 1024 * 1024);
  JobSystemSingleThreaded jobs(1024);
  for (int i = 0; i < 900; ++i) {
    bodies.GetPositionAndRotation(id, position, rotation);
    auto forward = rotation * Vec3::sAxisX();
    argos::SwarmDeckStep(system, id, position, rotation, forward * .10f, .10f);
    bodies.SetLinearAndAngularVelocity(
        id, Vec3(forward.GetX() * .25f, forward.GetY() * .25f,
                 bodies.GetLinearVelocity(id).GetZ()), Vec3::sZero());
    for (int j = 0; j < 10; ++j) system.Update(.001f, 1, &allocator, &jobs);
  }
  position = bodies.GetPosition(id);
  std::cout << "corrected=" << corrected << " wall=" << wall << " final_y=" << position.GetY() << '\n';
  if (wall) return position.GetY() >= 1.4f && position.GetY() <= 2.0f ? 0 : 1;
  return position.GetY() < 1.0f ? 0 : 1;
}

int main(int argc, char** argv) {
  if (argc < 2) return 2;
  RegisterDefaultAllocator();
  Factory::sInstance = new Factory();
  RegisterTypes();
  int result = traverse(argv[1], argc < 3 || std::string(argv[2]) != "legacy",
                        argc >= 3 && std::string(argv[2]) == "wall");
  UnregisterTypes();
  delete Factory::sInstance;
  return result;
}
