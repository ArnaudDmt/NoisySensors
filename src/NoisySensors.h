/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/GlobalPlugin.h>
#include <random>

namespace mc_plugin
{

struct NoisySensors : public mc_control::GlobalPlugin
{
  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override;

  void reset(mc_control::MCGlobalController & controller) override;

  void before(mc_control::MCGlobalController &) override;

  void after(mc_control::MCGlobalController & controller) override;

  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

  ~NoisySensors() override;

private:
  bool withGyroNoise_ = false;
  Eigen::Vector3d gyroStandardDev_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyroOffset_ = Eigen::Vector3d::Zero();

  bool withAcceleroNoise_ = false;
  Eigen::Vector3d acceleroStandardDev_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleroOffset_ = Eigen::Vector3d::Zero();

  bool withForceSensorNoise_ = false;
  Eigen::Vector3d forceSenStandardDev_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d forceSenOffset_ = Eigen::Vector3d::Zero();

  bool withTorqueSensorNoise_ = false;
  Eigen::Vector3d torqueSenStandardDev_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d torqueSenOffset_ = Eigen::Vector3d::Zero();

  std::unordered_map<std::string, Eigen::Vector3d> unbiasedGyroSignal_;
  bool logsAdded_ = false;
};

} // namespace mc_plugin
