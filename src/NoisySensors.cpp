#include "NoisySensors.h"

#include <mc_control/GlobalPluginMacros.h>

namespace mc_plugin
{

NoisySensors::~NoisySensors() = default;

void NoisySensors::init(mc_control::MCGlobalController & ctl, const mc_rtc::Configuration & config)
{
  mc_rtc::log::info("NoisySensors::init called with configuration:\n{}", config.dump(true, true));
  config("withGyroNoise", withGyroNoise_);
  config("gyroStandardDev", gyroStandardDev_);
  config("gyroOffset", gyroOffset_);

  config("withAcceleroNoise", withAcceleroNoise_);
  config("acceleroStandardDev", acceleroStandardDev_);
  config("AcceleroOffset", acceleroOffset_);

  config("withForceSensorNoise", withForceSensorNoise_);
  config("forceSenStandardDev", forceSenStandardDev_);
  config("forceSenOffset", forceSenOffset_);

  config("withTorqueSensorNoise", withTorqueSensorNoise_);
  config("torqueSenStandardDev", torqueSenStandardDev_);
  config("torqueSenOffset", torqueSenOffset_);
}

void NoisySensors::reset(mc_control::MCGlobalController & ctl)
{
  mc_rtc::log::info("NoisySensors::reset called");
}

void NoisySensors::before(mc_control::MCGlobalController & ctl)
{
  if(withGyroNoise_)
  {
    for(const auto & bodySensor : ctl.controller().robot().data()->bodySensors)
    {
      Eigen::Vector3d noisyMeasurement = bodySensor.angularVelocity();
      unbiasedGyroSignal_[bodySensor.name()] = noisyMeasurement;

      std::mt19937 generator(std::random_device{}());
      std::normal_distribution<double> dist_x(0.0, gyroStandardDev_.x());
      std::normal_distribution<double> dist_y(0.0, gyroStandardDev_.y());
      std::normal_distribution<double> dist_z(0.0, gyroStandardDev_.z());

      // Add Gaussian noise
      noisyMeasurement.x() += dist_x(generator);
      noisyMeasurement.y() += dist_y(generator);
      noisyMeasurement.z() += dist_z(generator);

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
      std::normal_distribution<double> dist_x(0.0, acceleroStandardDev_.x());
      std::normal_distribution<double> dist_y(0.0, acceleroStandardDev_.y());
      std::normal_distribution<double> dist_z(0.0, acceleroStandardDev_.z());

      // Add Gaussian noise
      noisyMeasurement.x() += dist_x(generator);
      noisyMeasurement.y() += dist_y(generator);
      noisyMeasurement.z() += dist_z(generator);

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
      std::normal_distribution<double> dist_x(0.0, forceSenStandardDev_.x());
      std::normal_distribution<double> dist_y(0.0, forceSenStandardDev_.y());
      std::normal_distribution<double> dist_z(0.0, forceSenStandardDev_.z());

      // Add Gaussian noise
      noisyMeasurement.x() += dist_x(generator);
      noisyMeasurement.y() += dist_y(generator);
      noisyMeasurement.z() += dist_z(generator);

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
      std::normal_distribution<double> dist_x(0.0, torqueSenStandardDev_.x());
      std::normal_distribution<double> dist_y(0.0, torqueSenStandardDev_.y());
      std::normal_distribution<double> dist_z(0.0, torqueSenStandardDev_.z());

      // Add Gaussian noise
      noisyMeasurement.x() += dist_x(generator);
      noisyMeasurement.y() += dist_y(generator);
      noisyMeasurement.z() += dist_z(generator);

      // Add an offset
      noisyMeasurement += torqueSenOffset_;

      sva::ForceVecd noisyTorque = sva::ForceVecd::Zero();
      noisyTorque.moment() = noisyMeasurement;

      std::map<std::string, sva::ForceVecd> sensorNoise = {{forceSensor.name(), noisyTorque}};
      ctl.setWrenches(sensorNoise);
    }
  }

  if(!logsAdded_ && withGyroNoise_)
  {
    auto & logger = ctl.controller().logger();
    for(const auto & bodySensor : ctl.controller().robot().data()->bodySensors)
    {
      logger.addLogEntry("NoisySensors_gyro_" + bodySensor.name() + "_biased",
                         [&bodySensor]() { return bodySensor.angularVelocity(); });
      logger.addLogEntry("NoisySensors_gyro_" + bodySensor.name() + "_unBiased",
                         [this, &bodySensor]() { return unbiasedGyroSignal_.at(bodySensor.name()); });
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
