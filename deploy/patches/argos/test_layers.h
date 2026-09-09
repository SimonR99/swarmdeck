// Shared layer filters for the standalone Jolt regression programs.
#pragma once
#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
using namespace JPH;
struct Layers : BroadPhaseLayerInterface {
  uint GetNumBroadPhaseLayers() const override { return 2; }
  BroadPhaseLayer GetBroadPhaseLayer(ObjectLayer l) const override { return BroadPhaseLayer(l); }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
  const char* GetBroadPhaseLayerName(BroadPhaseLayer) const override { return "test"; }
#endif
};
struct Pair : ObjectLayerPairFilter {
  bool ShouldCollide(ObjectLayer a, ObjectLayer b) const override { return a || b; }
};
struct BroadPair : ObjectVsBroadPhaseLayerFilter {
  bool ShouldCollide(ObjectLayer, BroadPhaseLayer) const override { return true; }
};
