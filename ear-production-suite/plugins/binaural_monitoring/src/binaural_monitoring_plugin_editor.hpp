#pragma once

#include "JuceHeader.h"

#include "../../shared/components/ear_button.hpp"
#include "../../shared/components/level_meter.hpp"
#include "../../shared/components/onboarding.hpp"
#include "../../shared/components/overlay.hpp"
#include "../../shared/components/ear_header.hpp"
#include "binaural_monitoring_plugin_processor.hpp"
#include "headphone_channel_meter.hpp"

class EarMonitoringAudioProcessorEditor
    : public AudioProcessorEditor,
      private ear::plugin::ui::Onboarding::Listener {
 public:
  EarMonitoringAudioProcessorEditor(EarMonitoringAudioProcessor*);
  ~EarMonitoringAudioProcessorEditor();

  void paint(Graphics&) override;
  void resized() override;

 private:
  EarMonitoringAudioProcessor* p_;

  std::unique_ptr<ear::plugin::ui::EarHeader> header_;
  std::unique_ptr<ear::plugin::ui::EarButton> onBoardingButton_;
  std::unique_ptr<ear::plugin::ui::Overlay> onBoardingOverlay_;
  std::unique_ptr<ear::plugin::ui::Onboarding> onBoardingContent_;

  std::vector<std::unique_ptr<ear::plugin::ui::HeadphoneChannelMeter>> headphoneMeters_;

  std::unique_ptr<InterProcessLock> propertiesFileLock_;
  std::unique_ptr<PropertiesFile> propertiesFile_;

  // --- Onboarding::Listener
  void endButtonClicked(ear::plugin::ui::Onboarding* onboarding) override;
  void moreButtonClicked(ear::plugin::ui::Onboarding* onboarding) override;

  JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(
      EarMonitoringAudioProcessorEditor)
};
