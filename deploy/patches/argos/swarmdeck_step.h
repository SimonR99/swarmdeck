// Collision-checked step assistance for simplified upright Jolt robot bodies.
// Full body sweeps check overhead clearance, the raised advance, and landing.
#pragma once
#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Body/BodyFilter.h>
#include <algorithm>
#include <cmath>

namespace argos {
inline void SwarmDeckStep(JPH::PhysicsSystem& system, const JPH::BodyID& id,
                         JPH::RVec3& position, const JPH::Quat& rotation,
                         JPH::Vec3 direction, float max_height) {
  auto& bodies = system.GetBodyInterface();
  // Do not turn falling/jumping bodies into flying platforms.
  if (direction.LengthSq() < 1e-8f || std::abs(bodies.GetLinearVelocity(id).GetZ()) > 0.3f) return;
  const auto shape = bodies.GetShape(id);
  JPH::IgnoreSingleBodyFilter filter(id);
  auto cast = [&](JPH::RVec3 start, JPH::Vec3 delta) {
    JPH::ClosestHitCollisionCollector<JPH::CastShapeCollector> hit;
    JPH::RShapeCast sweep(shape, JPH::Vec3::sReplicate(1.0f),
                         JPH::RMat44::sRotationTranslation(rotation, start), delta);
    JPH::ShapeCastSettings settings;
    settings.mUseShrunkenShapeAndConvexRadius = true;
    system.GetNarrowPhaseQuery().CastShape(sweep, settings,
        JPH::RVec3::sZero(), hit, {}, {}, filter);
    return hit;
  };
  // A small skin separates the horizontal cast from the supporting floor.
  const JPH::Vec3 skin(0, 0, 0.006f);
  auto obstacle = cast(position + skin, direction);
  if (!obstacle.HadHit()) return;
  auto support = cast(position + skin, JPH::Vec3(0, 0, -0.025f));
  if (!support.HadHit() ||
      (-support.mHit.mPenetrationAxis.Normalized()).GetZ() < 0.866f) return;
  const JPH::Vec3 up(0, 0, max_height + 0.012f);
  if (cast(position + skin, up).HadHit()) return;
  const auto raised = position + skin + up;
  if (cast(raised, direction).HadHit()) return;
  auto landing = cast(raised + direction, -(up + skin));
  if (!landing.HadHit() || bodies.GetMotionType(landing.mHit.mBodyID2) != JPH::EMotionType::Static) return;
  // Penetration axis points towards the obstacle: downward on a walkable top.
  const auto normal = -landing.mHit.mPenetrationAxis.Normalized();
  if (normal.GetZ() < 0.866f) return;
  const auto candidate = raised + direction - (up + skin) * landing.mHit.mFraction;
  const double rise = candidate.GetZ() - position.GetZ();
  if (rise < 0.015 || rise > max_height + 0.002) return;
  position = candidate + skin;
  bodies.SetPosition(id, position, JPH::EActivation::Activate);
}
}
