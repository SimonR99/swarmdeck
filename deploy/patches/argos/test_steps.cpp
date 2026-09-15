// Standalone collision regression using the same Jolt library as ARGoS.
#include "swarmdeck_step.h"
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <iostream>
#include <stdexcept>
using namespace JPH;
#include "test_layers.h"
void check(float height, float limit, bool expected, bool ceiling=false, bool dynamic=false, bool floor=true) {
  Layers layers; Pair pair; BroadPair broad;
  PhysicsSystem system; system.Init(32,0,64,64,layers,broad,pair);
  auto& bodies=system.GetBodyInterface();
  auto box=[&](Vec3 half, RVec3 center, bool moving) {
    BodyCreationSettings settings(new BoxShape(half, moving ? 0.04f : 0.005f),center,Quat::sIdentity(),moving?EMotionType::Dynamic:EMotionType::Static,moving?1:0);
    return bodies.CreateAndAddBody(settings,EActivation::Activate);
  };
  if(floor) box(Vec3(10,10,.1f),RVec3(0,0,-.1f),false);
  box(Vec3(.5f,2,height/2),RVec3(1.04f,0,height/2),dynamic);
  if(ceiling) box(Vec3(2,2,.1f),RVec3(0,0,.55f),false);
  RVec3 position(0,0,.2f);
  auto robot=box(Vec3(.5f,.25f,.2f),position,true);
  system.OptimizeBroadPhase();
  argos::SwarmDeckStep(system,robot,position,Quat::sIdentity(),Vec3(.10f,0,0),limit);
  bool climbed=position.GetZ()>.22f;
  std::cout<<"height="<<height<<" limit="<<limit<<" ceiling="<<ceiling<<" dynamic="<<dynamic<<" floor="<<floor<<" rise="<<position.GetZ()-.2f<<'\n';
  if(climbed!=expected) throw std::runtime_error("unexpected step result");
}
int main() {
  RegisterDefaultAllocator(); Factory::sInstance=new Factory(); RegisterTypes();
  check(.14f,.15f,true); check(.15f,.15f,true); check(.17f,.15f,false);
  check(.29f,.30f,true); check(.30f,.30f,true); check(.32f,.30f,false);
  check(.14f,.15f,false,true); check(.14f,.15f,false,false,true);
  check(.14f,.15f,false,false,false,false); check(1.f,.30f,false);
  UnregisterTypes(); delete Factory::sInstance;
}
