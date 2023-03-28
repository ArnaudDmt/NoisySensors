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
  bool withAcceleroNoise_ = false;
  bool withForceSensorNoise_ = false;
  bool withTorqueSensorNoise_ = false;

  double gyroStandardDev_ = 0.1;
  double acceleroStandardDev_ = 0.1;
  double forceSenStandardDev_ = 0.1;
  double torqueSenStandardDev_ = 0.1;
};

} // namespace mc_plugin
