#pragma once

#include "JuceHeader.h"

#include "hoa_backend.hpp"
#include "../../shared/components/level_meter_calculator.hpp"
#include "common_definition_helper.h"

#include <memory>

namespace ear {
namespace plugin {
namespace ui {
class HoaJuceFrontendConnector;
}
class HoaBackend;
}  // namespace plugin
}  // namespace ear

class HoaAudioProcessor : public AudioProcessor {
 public:
  HoaAudioProcessor();
  ~HoaAudioProcessor();

  void prepareToPlay(double sampleRate, int samplesPerBlock) override;
  void releaseResources() override;

  bool isBusesLayoutSupported(const BusesLayout& layouts) const override;

  void processBlock(AudioBuffer<float>&, MidiBuffer&) override;

  AudioProcessorEditor* createEditor() override;
  bool hasEditor() const override;

  const String getName() const override;

  bool acceptsMidi() const override;
  bool producesMidi() const override;
  bool isMidiEffect() const override;
  double getTailLengthSeconds() const override;

  int getNumPrograms() override;
  int getCurrentProgram() override;
  void setCurrentProgram(int index) override;
  const String getProgramName(int index) override;
  void changeProgramName(int index, const String& newName) override;

  void getStateInformation(MemoryBlock& destData) override;
  void setStateInformation(const void* data, int sizeInBytes) override;

  void updateTrackProperties(const TrackProperties& properties) override;

  AdmCommonDefinitionHelper admCommonDefinitions{};  // ME added

  AudioParameterInt* getRouting() { return routing_; }

  AudioParameterInt* getHoaType() { return hoaTypeIndex_; }  // ME added, similar to DS

  
  AudioProcessorParameter* getBypassParameter() {
    return bypass_;
  }

  std::weak_ptr<ear::plugin::LevelMeterCalculator> getLevelMeter() {
    return levelMeter_;
  };

  ear::plugin::ui::HoaJuceFrontendConnector* getFrontendConnector() {
    return connector_.get();
  }
  //ME add not sure if good
  //void setNumHoaTypes(int numHoaTypes);
  //ME end


 private:
  ear::plugin::communication::ConnectionId connectionId_;

  AudioParameterInt* routing_;
  AudioParameterInt* hoaTypeIndex_;//ME add, similar to DS
  AudioParameterBool* bypass_;
  //AudioParameterInt* numHoaTypes_;//ME added, not sure if good

  std::unique_ptr<ear::plugin::ui::HoaJuceFrontendConnector>
      connector_;
  std::unique_ptr<ear::plugin::HoaBackend> backend_;

  int samplerate_;
  std::shared_ptr<ear::plugin::LevelMeterCalculator> levelMeter_;

  JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(HoaAudioProcessor)
};
