#pragma once
#include "ui/binaural_monitoring_frontend_backend_connector.hpp"

#include "JuceHeader.h"

#include "../../shared/components/orientation.hpp"
//#include "../../shared/components/ear_button.hpp"
//#include "../../shared/components/ear_combo_box.hpp"
//#include "../../shared/components/panner_top_view.hpp"
//#include "../../shared/components/panner_side_view.hpp"
#include "../../shared/helper/multi_async_updater.h"
#include "binaural_monitoring_plugin_processor.hpp"
#include <memory>

namespace ear {
namespace plugin {
namespace ui {

class OrientationView;

class EarNameTextEditor;
class EarSlider;
class EarInvertedSlider;
class PannerTopView;
class PannerSideView;

class BinauralMonitoringJuceFrontendConnector
    : public ear::plugin::ui::BinauralMonitoringFrontendBackendConnector,
      private AudioProcessorParameter::Listener,
      //TextEditor::Listener,
      //Slider::Listener,
      //Button::Listener,
      //ear::plugin::ui::PannerTopView::Listener,
      //ear::plugin::ui::PannerSideView::Listener,
      //ear::plugin::ui::EarComboBox::Listener {
      ear::plugin::ui::OrientationView::Listener {

public:
  /**
   * Note: Make sure that all AudioProcessorParameters are already
   * added to the ObjectsAudioProcessor at the time the ctor
   * is called.
   */
   BinauralMonitoringJuceFrontendConnector(EarBinauralMonitoringAudioProcessor* processor);
  ~BinauralMonitoringJuceFrontendConnector();

  void parameterValueChanged(int parameterIndex, float newValue) override;
  void parameterGestureChanged(int parameterIndex,
                               bool gestureIsStarting) override{};

  // Orientation Controls
  void setYawView(std::shared_ptr<OrientationView> view);
  void setPitchView(std::shared_ptr<OrientationView> view);
  void setRollView(std::shared_ptr<OrientationView> view);

  // Value Setters
  void setYaw(float yaw);
  void setPitch(float pitch);
  void setRoll(float roll);

  /*
  // Main Value Box
  void setStatusBarLabel(std::shared_ptr<Label> statusBarLabel);
  void setNameTextEditor(std::shared_ptr<EarNameTextEditor> nameTextEditor);
  void setColourComboBox(std::shared_ptr<EarComboBox> colourComboBox);
  void setRoutingComboBox(std::shared_ptr<EarComboBox> routingComboBox);

  // Panning Value Box
  void setGainSlider(std::shared_ptr<EarSlider> gainSlider);
  void setAzimuthSlider(std::shared_ptr<EarInvertedSlider> azimuthSlider);
  void setElevationSlider(std::shared_ptr<EarSlider> elevationSlider);
  void setDistanceSlider(std::shared_ptr<EarSlider> distanceSlider);

  // Extent Value Box
  void setLinkSizeButton(std::shared_ptr<EarButton> button);
  void setSizeLabel(std::shared_ptr<Label> label);
  void setSizeSlider(std::shared_ptr<EarSlider> slider);
  void setWidthLabel(std::shared_ptr<Label> label);
  void setWidthSlider(std::shared_ptr<EarSlider> slider);
  void setHeightLabel(std::shared_ptr<Label> label);
  void setHeightSlider(std::shared_ptr<EarSlider> slider);
  void setDepthLabel(std::shared_ptr<Label> label);
  void setDepthSlider(std::shared_ptr<EarSlider> slider);
  void setDiffuseSlider(std::shared_ptr<EarSlider> slider);
  void setDivergenceButton(std::shared_ptr<EarButton> button);
  void setFactorLabel(std::shared_ptr<Label> label);
  void setFactorSlider(std::shared_ptr<EarSlider> slider);
  void setRangeLabel(std::shared_ptr<Label> label);
  void setRangeSlider(std::shared_ptr<EarSlider> slider);

  // Panning View Value Box
  void setPannerTopView(std::shared_ptr<PannerTopView> panner);
  void setPannerSideView(std::shared_ptr<PannerSideView> panner);

  // Value setter
  void setName(const std::string& name);
  void setColour(Colour colour);
  void setRouting(int routing);
  void setGain(float gain);

  void setAzimuth(float azimuth);
  void setElevation(float elevation);
  void setDistance(float distance);

  void setLinkSize(bool value);
  void setSize(float value);
  void setWidth(float value);
  void setHeight(float value);
  void setDepth(float value);
  void setDiffuse(float value);
  void setDivergence(bool value);
  void setFactor(float value);
  void setRange(float value);
  */

protected:
  // Orientation::Listener
  void orientationValueChanged(ear::plugin::ui::OrientationView* view) override;
  void orientationDragStarted(ear::plugin::ui::OrientationView* view) override;
  void orientationDragEnded(ear::plugin::ui::OrientationView* view) override;

  /*
  // Slider::Listener
  void sliderValueChanged(Slider* slider) override;
  void sliderDragStarted(Slider*) override;
  void sliderDragEnded(Slider*) override;

  // PannerTopView::Listener
  void pannerValueChanged(ear::plugin::ui::PannerTopView* panner) override;
  void pannerDragStarted(ear::plugin::ui::PannerTopView* panner) override;
  void pannerDragEnded(ear::plugin::ui::PannerTopView* panner) override;

  // PannerSideView::Listener
  void pannerValueChanged(ear::plugin::ui::PannerSideView* panner) override;
  void pannerDragStarted(ear::plugin::ui::PannerSideView* panner) override;
  void pannerDragEnded(ear::plugin::ui::PannerSideView* panner) override;

  // EarComboBox::Listener
  void comboBoxChanged(
      ear::plugin::ui::EarComboBox* comboBoxThatHasChanged) override;

  // Button::Listener
  void buttonClicked(Button*) override;
  */

private:
  EarBinauralMonitoringAudioProcessor* p_;
  std::map<int, RangedAudioParameter*> parameters_;

  // Orientation Controls
  std::weak_ptr<OrientationView> yawControl_;
  std::weak_ptr<OrientationView> pitchControl_;
  std::weak_ptr<OrientationView> rollControl_;

  /*
  // Main Value Box
  std::weak_ptr<EarComboBox> colourComboBox_;
  std::weak_ptr<EarComboBox> routingComboBox_;
  std::weak_ptr<EarNameTextEditor> nameTextEditor_;

  // Panning Value Box
  std::weak_ptr<EarSlider> gainSlider_;
  std::weak_ptr<EarInvertedSlider> azimuthSlider_;
  std::weak_ptr<EarSlider> elevationSlider_;
  std::weak_ptr<EarSlider> distanceSlider_;

  // Extent Value Box
  std::weak_ptr<EarButton> linkSizeButton_;
  std::weak_ptr<Label> sizeLabel_;
  std::weak_ptr<EarSlider> sizeSlider_;
  std::weak_ptr<Label> widthLabel_;
  std::weak_ptr<EarSlider> widthSlider_;
  std::weak_ptr<Label> heightLabel_;
  std::weak_ptr<EarSlider> heightSlider_;
  std::weak_ptr<Label> depthLabel_;
  std::weak_ptr<EarSlider> depthSlider_;
  std::weak_ptr<EarSlider> diffuseSlider_;
  std::weak_ptr<EarButton> divergenceButton_;
  std::weak_ptr<Label> factorLabel_;
  std::weak_ptr<EarSlider> factorSlider_;
  std::weak_ptr<Label> rangeLabel_;
  std::weak_ptr<EarSlider> rangeSlider_;

  // Panning View Value Box
  std::weak_ptr<PannerTopView> pannerTopView_;
  std::weak_ptr<PannerSideView> pannerSideView_;

  std::weak_ptr<Label> statusBar_;
  */

  MultiAsyncUpdater updater_;

  // Values
  double cachedYaw_;
  double cachedPitch_;
  double cachedRoll_;

  /*
  // Values
  Colour cachedColour_;
  int cachedRouting_;
  std::string cachedName_;
  float cachedGain_;
  float cachedAzimuth_;
  float cachedElevation_;
  float cachedDistance_;
  bool cachedLinkSize_;
  float cachedSize_;
  float cachedWidth_;
  float cachedHeight_;
  float cachedDepth_;
  float cachedDiffuse_;
  bool cachedDivergence_;
  float cachedFactor_;
  float cachedRange_;
  std::string cachedStatusBarText_;
  */
};

}  // namespace ui
}  // namespace plugin
}  // namespace ear
