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
  config("withAcceleroNoise", withAcceleroNoise_);
  config("acceleroStandardDev", acceleroStandardDev_);
  config("withForceSensorNoise", withForceSensorNoise_);
  config("forceSenStandardDev", forceSenStandardDev_);
  config("withTorqueSensorNoise", withTorqueSensorNoise_);
  config("torqueSenStandardDev", torqueSenStandardDev_);
}

void NoisySensors::reset(mc_control::MCGlobalController & ctl)
{
  mc_rtc::log::info("NoisySensors::reset called");
}

void NoisySensors::before(mc_control::MCGlobalController & ctl)
{
  mc_rtc::log::info("NoisySensors::before");
  const mc_rbdyn::BodySensorVector & bodySensors = ctl.get_robot_module()->bodySensors();
  const std::vector<mc_rbdyn::ForceSensor> & forceSensors = ctl.get_robot_module()->forceSensors();

  if(withGyroNoise_)
  {
    for(const auto & bodySensor : bodySensors)
    {
      Eigen::Vector3d noisyMeasurement = bodySensor.angularVelocity();
      std::mt19937 generator(std::random_device{}());
      std::normal_distribution<double> dist(0.0, gyroStandardDev_);

      // Add Gaussian noise
      for(int i = 0; i < noisyMeasurement.size(); i++)
      {
        noisyMeasurement(i) += dist(generator);
      }
      std::map<std::string, Eigen::Vector3d> sensorNoise = {{bodySensor.name(), noisyMeasurement}};
      ctl.setSensorAngularVelocities(sensorNoise);
    }
  }

  if(withAcceleroNoise_)
  {
    for(const auto & bodySensor : bodySensors)
    {
      Eigen::Vector3d noisyMeasurement = bodySensor.linearAcceleration();
      std::mt19937 generator(std::random_device{}());
      std::normal_distribution<double> dist(0.0, gyroStandardDev_);

      // Add Gaussian noise
      for(int i = 0; i < noisyMeasurement.size(); i++)
      {
        noisyMeasurement(i) += dist(generator);
      }
      std::map<std::string, Eigen::Vector3d> sensorNoise = {{bodySensor.name(), noisyMeasurement}};
      ctl.setSensorAccelerations(sensorNoise);
    }
  }

  if(withForceSensorNoise_)
  {
    for(const auto & forceSensor : forceSensors)
    {
      Eigen::Vector3d noisyMeasurement = forceSensor.force();
      std::mt19937 generator(std::random_device{}());
      std::normal_distribution<double> dist(0.0, gyroStandardDev_);

      // Add Gaussian noise
      for(int i = 0; i < noisyMeasurement.size(); i++)
      {
        noisyMeasurement(i) += dist(generator);
      }
      sva::ForceVecd noisyForce = sva::ForceVecd::Zero();
      noisyForce.force() = noisyMeasurement;

      std::map<std::string, sva::ForceVecd> sensorNoise = {{forceSensor.name(), noisyForce}};
      ctl.setWrenches(sensorNoise);
    }
  }

  if(withTorqueSensorNoise_)
  {
    for(const auto & forceSensor : forceSensors)
    {
      Eigen::Vector3d noisyMeasurement = forceSensor.couple();
      std::mt19937 generator(std::random_device{}());
      std::normal_distribution<double> dist(0.0, gyroStandardDev_);

      // Add Gaussian noise
      for(int i = 0; i < noisyMeasurement.size(); i++)
      {
        noisyMeasurement(i) += dist(generator);
      }
      sva::ForceVecd noisyTorque = sva::ForceVecd::Zero();
      noisyTorque.moment() = noisyMeasurement;

      std::map<std::string, sva::ForceVecd> sensorNoise = {{forceSensor.name(), noisyTorque}};
      ctl.setWrenches(sensorNoise);
    }
  }
}

void NoisySensors::after(mc_control::MCGlobalController & ctl)
{
  mc_rtc::log::info("NoisySensors::after");
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
