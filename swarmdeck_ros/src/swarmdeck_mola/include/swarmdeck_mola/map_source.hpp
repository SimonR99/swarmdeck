#pragma once

#include <mola_kernel/interfaces/ExecutableBase.h>
#include <mola_kernel/interfaces/MapSourceBase.h>

#include <memory>

namespace swarmdeck_mola
{
/** Loadable MOLA map source with externally corrected keyframe poses.
 *
 * Each instance selects one component from a coherent onboard snapshot. The framework owns
 * its execution thread; this module never starts a ROS node or publishes TF.
 * Published maps are immutable, and unavailable input retracts the visible
 * layer with explicit availability metadata rather than advertising old data
 * as a fresh map. Geometry alone does not certify free space for planning.
 */
class SwarmDeckMapSource final : public mola::ExecutableBase, public mola::MapSourceBase
{
  DEFINE_MRPT_OBJECT(SwarmDeckMapSource, swarmdeck_mola)

 public:
  SwarmDeckMapSource();
  ~SwarmDeckMapSource() override;

  void initialize(const mola::Yaml& config) override;
  void spinOnce() override;
  void onQuit() override;

 private:
  struct State;
  std::unique_ptr<State> state_;
  void unavailable(const std::string& reason);
};
}  // namespace swarmdeck_mola
