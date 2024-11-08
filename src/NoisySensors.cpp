#include "NoisySensors.h"

#include <mc_control/GlobalPluginMacros.h>

namespace mc_plugin
{

NoisySensors::~NoisySensors() = default;

void NoisySensors::init(mc_control::MCGlobalController & ctl, const mc_rtc::Configuration & config)
{
  mc_rtc::log::info("NoisySensors::init called with configuration:\n{}", config.dump(true, true));
  config("withNoisyGyro", withNoisyGyro_);
  config("gyroNoise_StdDev", gyroNoise_StdDev_);
  config("gyroRandomWalk_StdDev", gyroRandomWalk_StdDev_);
  config("gyroOffset", gyroOffset_);

  config("withAcceleroNoise", withAcceleroNoise_);
  config("acceleroNoise_StdDev", acceleroNoise_StdDev_);
  config("acceleroRandomWalk_StdDev", acceleroRandomWalk_StdDev_);
  config("AcceleroOffset", acceleroOffset_);

  config("withForceSensorNoise", withForceSensorNoise_);
  config("forceSenNoise_StdDev", forceSenNoise_StdDev_);
  config("forceSenRandomWalk_StdDev", forceSenRandomWalk_StdDev_);
  config("forceSenOffset", forceSenOffset_);

  config("withTorqueSensorNoise", withTorqueSensorNoise_);
  config("torqueSenNoise_StdDev", torqueSenNoise_StdDev_);
  config("torqueSenRandomWalk_StdDev", torqueSenRandomWalk_StdDev_);
  config("torqueSenOffset", torqueSenOffset_);
}

void NoisySensors::reset(mc_control::MCGlobalController & ctl)
{
  mc_rtc::log::info("NoisySensors::reset called");
}

void NoisySensors::before(mc_control::MCGlobalController & ctl)
{
  if(withNoisyGyro_)
  {
    for(const auto & bodySensor : ctl.controller().robot().data()->bodySensors)
    {
      Eigen::Vector3d noisyMeasurement = bodySensor.angularVelocity();
      unbiasedGyroSignal_[bodySensor.name()] = noisyMeasurement;

      std::mt19937 generator(std::random_device{}());
      std::normal_distribution<double> noise_x(0.0, gyroNoise_StdDev_.x());
      std::normal_distribution<double> noise_y(0.0, gyroNoise_StdDev_.y());
      std::normal_distribution<double> noise_z(0.0, gyroNoise_StdDev_.z());

      // Add Gaussian noise
      noisyMeasurement.x() += noise_x(generator);
      noisyMeasurement.y() += noise_y(generator);
      noisyMeasurement.z() += noise_z(generator);

      // Random walk for the bias
      std::normal_distribution<double> random_walk_x(0.0, gyroRandomWalk_StdDev_.x());
      std::normal_distribution<double> random_walk_y(0.0, gyroRandomWalk_StdDev_.y());
      std::normal_distribution<double> random_walk_z(0.0, gyroRandomWalk_StdDev_.z());

      // Update the bias with a random walk
      gyroOffset_.x() += random_walk_x(generator);
      gyroOffset_.y() += random_walk_y(generator);
      gyroOffset_.z() += random_walk_z(generator);

      // Add an offset
      noisyMeasurement += gyroOffset_;

      std::map<std::string, Eigen::Vector3d> sensorNoise = {{bodySensor.name(), noisyMeasurement}};
      ctl.setSensorAngularVelocities(sensorNoise);
    }
  }

  if(withAcceleroNoise_)
  {
    for(const auto & bodySensor : ctl.controller().robot().data()->bodySensors)
    {
      Eigen::Vector3d noisyMeasurement = bodySensor.linearAcceleration();
      std::mt19937 generator(std::random_device{}());
      std::normal_distribution<double> noise_x(0.0, acceleroNoise_StdDev_.x());
      std::normal_distribution<double> noise_y(0.0, acceleroNoise_StdDev_.y());
      std::normal_distribution<double> noise_z(0.0, acceleroNoise_StdDev_.z());

      // Add Gaussian noise
      noisyMeasurement.x() += noise_x(generator);
      noisyMeasurement.y() += noise_y(generator);
      noisyMeasurement.z() += noise_z(generator);

      // Random walk for the bias
      std::normal_distribution<double> random_walk_x(0.0, acceleroRandomWalk_StdDev_.x());
      std::normal_distribution<double> random_walk_y(0.0, acceleroRandomWalk_StdDev_.y());
      std::normal_distribution<double> random_walk_z(0.0, acceleroRandomWalk_StdDev_.z());

      // Update the bias with a random walk
      acceleroOffset_.x() += random_walk_x(generator);
      acceleroOffset_.y() += random_walk_y(generator);
      acceleroOffset_.z() += random_walk_z(generator);

      // Add an offset
      noisyMeasurement += acceleroOffset_;

      std::map<std::string, Eigen::Vector3d> sensorNoise = {{bodySensor.name(), noisyMeasurement}};
      ctl.setSensorLinearAccelerations(sensorNoise);
    }
  }

  if(withForceSensorNoise_)
  {
    for(const auto & forceSensor : ctl.controller().robot().data()->forceSensors)
    {
      Eigen::Vector3d noisyMeasurement = forceSensor.force();
      std::mt19937 generator(std::random_device{}());
      std::normal_distribution<double> noise_x(0.0, forceSenNoise_StdDev_.x());
      std::normal_distribution<double> noise_y(0.0, forceSenNoise_StdDev_.y());
      std::normal_distribution<double> noise_z(0.0, forceSenNoise_StdDev_.z());

      // Add Gaussian noise
      noisyMeasurement.x() += noise_x(generator);
      noisyMeasurement.y() += noise_y(generator);
      noisyMeasurement.z() += noise_z(generator);

      // Random walk for the bias
      std::normal_distribution<double> random_walk_x(0.0, forceSenRandomWalk_StdDev_.x());
      std::normal_distribution<double> random_walk_y(0.0, forceSenRandomWalk_StdDev_.y());
      std::normal_distribution<double> random_walk_z(0.0, forceSenRandomWalk_StdDev_.z());

      // Update the bias with a random walk
      forceSenOffset_.x() += random_walk_x(generator);
      forceSenOffset_.y() += random_walk_y(generator);
      forceSenOffset_.z() += random_walk_z(generator);

      // Add an offset
      noisyMeasurement += forceSenOffset_;

      sva::ForceVecd noisyForce = sva::ForceVecd::Zero();
      noisyForce.force() = noisyMeasurement;

      std::map<std::string, sva::ForceVecd> sensorNoise = {{forceSensor.name(), noisyForce}};
      ctl.setWrenches(sensorNoise);
    }
  }

  if(withTorqueSensorNoise_)
  {
    for(const auto & forceSensor : ctl.controller().robot().data()->forceSensors)
    {
      Eigen::Vector3d noisyMeasurement = forceSensor.couple();
      std::mt19937 generator(std::random_device{}());
      std::normal_distribution<double> noise_x(0.0, torqueSenNoise_StdDev_.x());
      std::normal_distribution<double> noise_y(0.0, torqueSenNoise_StdDev_.y());
      std::normal_distribution<double> noise_z(0.0, torqueSenNoise_StdDev_.z());

      // Add Gaussian noise
      noisyMeasurement.x() += noise_x(generator);
      noisyMeasurement.y() += noise_y(generator);
      noisyMeasurement.z() += noise_z(generator);

      // Random walk for the bias
      std::normal_distribution<double> random_walk_x(0.0, torqueSenRandomWalk_StdDev_.x());
      std::normal_distribution<double> random_walk_y(0.0, torqueSenRandomWalk_StdDev_.y());
      std::normal_distribution<double> random_walk_z(0.0, torqueSenRandomWalk_StdDev_.z());

      // Update the bias with a random walk
      torqueSenOffset_.x() += random_walk_x(generator);
      torqueSenOffset_.y() += random_walk_y(generator);
      torqueSenOffset_.z() += random_walk_z(generator);

      // Add an offset
      noisyMeasurement += torqueSenOffset_;

      sva::ForceVecd noisyTorque = sva::ForceVecd::Zero();
      noisyTorque.moment() = noisyMeasurement;

      std::map<std::string, sva::ForceVecd> sensorNoise = {{forceSensor.name(), noisyTorque}};
      ctl.setWrenches(sensorNoise);
    }
  }

  if(!logsAdded_ && withNoisyGyro_)
  {
    auto & logger = ctl.controller().logger();
    for(const auto & bodySensor : ctl.controller().robot().data()->bodySensors)
    {
      logger.addLogEntry("NoisySensors_gyro_" + bodySensor.name() + "_biased",
                         [&bodySensor]() { return bodySensor.angularVelocity(); });
      logger.addLogEntry("NoisySensors_gyro_" + bodySensor.name() + "_unBiased",
                         [this, &bodySensor]() { return unbiasedGyroSignal_.at(bodySensor.name()); });
      logger.addLogEntry("NoisySensors_gyro_" + bodySensor.name() + "_bias",
                         [this, &bodySensor]() -> Eigen::Vector3d
                         { return bodySensor.angularVelocity() - unbiasedGyroSignal_.at(bodySensor.name()); });
    }

    logsAdded_ = true;
  }
}

void NoisySensors::after(mc_control::MCGlobalController & ctl)
{
  // std::cout << ctl.controller().robot("hrp5_p").bodySensor("Accelerometer").angularVelocity() << std::endl;

  for(const auto & bodySensor : ctl.controller().robot().data()->bodySensors)
  {
    // std::cout << bodySensor.angularVelocity() << std::endl;
  }
}

mc_control::GlobalPlugin::GlobalPluginConfiguration NoisySensors::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = true;
  out.should_always_run = true;
  return out;
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("NoisySensors", mc_plugin::NoisySensors)
