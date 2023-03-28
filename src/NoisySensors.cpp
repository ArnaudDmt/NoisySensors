#include "NoisySensors.h"

#include <mc_control/GlobalPluginMacros.h>

namespace mc_plugin
{

NoisySensors::~NoisySensors() = default;

void NoisySensors::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  mc_rtc::log::info("NoisySensors::init called with configuration:\n{}", config.dump(true, true));
}

void NoisySensors::reset(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("NoisySensors::reset called");
}

void NoisySensors::before(mc_control::MCGlobalController &)
{
  mc_rtc::log::info("NoisySensors::before");
}

void NoisySensors::after(mc_control::MCGlobalController & controller)
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
