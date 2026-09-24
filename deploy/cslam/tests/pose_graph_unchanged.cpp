// Build/run against the pinned, patched C-SLAM headers and Jazzy GTSAM.
#include <cslam/back_end/unchanged_graph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <cassert>
#include <iostream>

int main() {
  cslam::UnchangedGraph cache;
  auto graph = boost::make_shared<gtsam::NonlinearFactorGraph>();
  auto values = boost::make_shared<gtsam::Values>();
  auto noise = gtsam::noiseModel::Isotropic::Sigma(6, 1.0);
  values->insert(0, gtsam::Pose3());
  values->insert(1, gtsam::Pose3());
  graph->emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      0, 1, gtsam::Pose3(), noise);
  assert(!cache.matches(graph, values, {0, 0}, 0));
  cache.remember(graph, values, {0, 0}, 0, *values);
  int skipped = 0;
  for (int i = 0; i < 100; ++i) {
    auto collected_graph = boost::make_shared<gtsam::NonlinearFactorGraph>();
    auto collected_values = boost::make_shared<gtsam::Values>(*values);
    collected_graph->emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        0, 1, gtsam::Pose3(), gtsam::noiseModel::Isotropic::Sigma(6, 1.0));
    skipped += cache.matches(collected_graph, collected_values, {0, 0}, 0);
  }
  assert(skipped == 100);
  auto remote_graph = boost::make_shared<gtsam::NonlinearFactorGraph>(*graph);
  auto remote_values = boost::make_shared<gtsam::Values>(*values);
  remote_values->insert(100, gtsam::Pose3());
  assert(!cache.matches(graph, remote_values, {0, 0}, 0));
  remote_graph->emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      1, 100, gtsam::Pose3(), noise);
  assert(!cache.matches(remote_graph, values, {0, 0}, 0));
  assert(!cache.matches(graph, values, {0, 1}, 0));
  assert(!cache.matches(graph, values, {0, 0}, 1));
  remote_graph = boost::make_shared<gtsam::NonlinearFactorGraph>();
  remote_graph->emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      0, 1, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)), noise);
  assert(!cache.matches(remote_graph, values, {0, 0}, 0));
  std::cout << "100/100 unchanged solves skipped; remote changes invalidate\n";
}
