#include <cslam/back_end/decentralized_pgo.h>
#include <cassert>
#include <iostream>

using namespace cslam;

std::shared_ptr<rclcpp::Node> make_node(bool residual) {
  auto node = std::make_shared<rclcpp::Node>("graph_rounds_test", "/r0");
  node->declare_parameter<int>("max_nb_robots", 1);
  node->declare_parameter<int>("robot_id", 0);
  node->declare_parameter<int64_t>("swarmdeck.map_epoch", 0);
  node->declare_parameter<std::string>("swarmdeck.epoch_state_path", "");
  node->declare_parameter<std::string>("swarmdeck.mission_id", "native-test");
  node->declare_parameter<int>("backend.pose_graph_optimization_start_period_ms", 100000);
  node->declare_parameter<int>("backend.pose_graph_optimization_loop_period_ms", 100000);
  node->declare_parameter<int>("backend.max_waiting_time_sec", 100);
  node->declare_parameter<bool>("backend.enable_broadcast_tf_frames", false);
  node->declare_parameter<double>("backend.upright_prior_sigma_rad", residual ? 0.01 : 0.0);
  node->declare_parameter<double>("backend.upright_prior_level_max_rad", 0.05);
  node->declare_parameter<double>("neighbor_management.heartbeat_period_sec", 100.0);
  node->declare_parameter<bool>("evaluation.enable_logs", false);
  node->declare_parameter<std::string>("evaluation.log_folder", "");
  node->declare_parameter<bool>("evaluation.enable_gps_recording", false);
  node->declare_parameter<bool>("evaluation.enable_simulated_rendezvous", false);
  node->declare_parameter<std::string>("evaluation.rendezvous_schedule_file", "");
  node->declare_parameter<bool>("evaluation.enable_pose_timestamps_recording", false);
  node->declare_parameter<bool>("visualization.enable", false);
  node->declare_parameter<int>("visualization.publishing_period_ms", 100000);
  return node;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  const bool residual = argc > 1 && std::string(argv[1]) == "residual";
  auto node = make_node(residual);
  DecentralizedPGO manager(node);
  const gtsam::LabeledSymbol first(GRAPH_LABEL, ROBOT_LABEL(0), 0);
  const gtsam::LabeledSymbol second(GRAPH_LABEL, ROBOT_LABEL(0), 1);
  manager.odometry_pose_estimates_->insert(first, gtsam::Pose3());
  manager.odometry_pose_estimates_->insert(second, gtsam::Pose3());
  *manager.current_pose_estimates_ = *manager.odometry_pose_estimates_;
  manager.pose_graph_->emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      first, second, gtsam::Pose3(), manager.default_noise_model_);

  if (residual) {
    const gtsam::LabeledSymbol third(GRAPH_LABEL, ROBOT_LABEL(0), 2);
    manager.odometry_pose_estimates_->clear();
    for (int i = 0; i < 3; ++i) {
      manager.odometry_pose_estimates_->insert(
          gtsam::LabeledSymbol(GRAPH_LABEL, ROBOT_LABEL(0), i),
          gtsam::Pose3(gtsam::Rot3::RzRyRx(0.02, 0.01, 0.03),
                       gtsam::Point3(i, 0, 0)));
    }
    *manager.current_pose_estimates_ = *manager.odometry_pose_estimates_;
    manager.pose_graph_->resize(0);
    for (int i = 0; i < 2; ++i) {
      manager.pose_graph_->emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          gtsam::LabeledSymbol(GRAPH_LABEL, ROBOT_LABEL(0), i),
          gtsam::LabeledSymbol(GRAPH_LABEL, ROBOT_LABEL(0), i + 1),
          gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)),
          manager.default_noise_model_);
    }
    manager.pose_graph_->emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        first, third,
        gtsam::Pose3(gtsam::Rot3::RzRyRx(0.01, 0, 0.02), gtsam::Point3(1.9, 0.1, 0.03)),
        manager.default_noise_model_);
  }
  const auto original_prior = manager.current_pose_estimates_->at<gtsam::Pose3>(first);

  using Result = cslam_common_interfaces::msg::OptimizationResult;
  std::vector<Result> reports;
  auto subscription = node->create_subscription<Result>(
      "/r0/cslam/optimized_estimates", 100,
      [&reports](Result::ConstSharedPtr msg) { reports.push_back(*msg); });
  for (int i = 0; i < 100 && subscription->get_publisher_count() == 0; ++i) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  assert(subscription->get_publisher_count() > 0);
  for (unsigned int round = 0; round < 3; ++round) {
    manager.start_optimization();
    if (manager.optimizer_state_ == OptimizerState::OPTIMIZATION) {
      manager.optimization_result_.wait();
      manager.check_result_and_finish_optimization();
    }
    for (int i = 0; i < 100 && reports.size() <= round; ++i) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    assert(reports.size() == round + 1);
    assert(reports.back().solution_clock == round + 1);
    assert(manager.optimization_count_ == 1);
    if (residual) {
      // Real report conversion and the subscriber update feed the next prior.
      const auto feedback = values_msg_to_gtsam(reports.back().estimates);
      assert(manager.current_pose_estimates_->equals(*feedback, 0.0));
      assert(!original_prior.equals(feedback->at<gtsam::Pose3>(first), 1e-9));
      assert(manager.pose_graph_->error(*feedback) > 1e-6);
      if (round == 0)
        assert(manager.aggregate_pose_graph_.first->size() == 7); // edges + prior + upright
    }
    assert(values_msg_to_gtsam(reports.back().estimates)->equals(
        *values_msg_to_gtsam(reports.front().estimates), 0.0));
  }
  std::cout << "3 rounds: 1 solve, 3 published results with increasing clocks; residual="
            << residual << "\n";
  rclcpp::shutdown();
}
