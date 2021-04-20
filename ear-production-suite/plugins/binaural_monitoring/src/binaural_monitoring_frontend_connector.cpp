#include "binaural_monitoring_frontend_connector.hpp"

//#include "../../shared/components/ear_combo_box.hpp"
//#include "../../shared/components/ear_name_text_editor.hpp"
#include "../../shared/components/ear_slider.hpp"
#include "../../shared/components/ear_inverted_slider.hpp"
//#include "../../shared/components/panner_top_view.hpp"
//#include "../../shared/components/panner_side_view.hpp"

namespace ear {
namespace plugin {
namespace ui {

//inline bool clipToBool(float value) { return value < 0.5 ? false : true; }

BinauralMonitoringJuceFrontendConnector::BinauralMonitoringJuceFrontendConnector(
  EarBinauralMonitoringAudioProcessor* processor)
    : BinauralMonitoringFrontendBackendConnector(), p_(processor) {
  if (p_) {
    auto& parameters = p_->getParameters();
    for (int i = 0; i < parameters.size(); ++i) {
      if (parameters[i]) {
        auto rangedParameter =
            dynamic_cast<RangedAudioParameter*>(parameters[i]);
        if (rangedParameter) {
          parameters_[i] = rangedParameter;
          rangedParameter->addListener(this);
        }
      }
    }
  }
}

BinauralMonitoringJuceFrontendConnector::~BinauralMonitoringJuceFrontendConnector() {
  for (auto parameter : parameters_) {
    if (parameter.second) {
      parameter.second->removeListener(this);
    }
  }

  if (auto orientationControl = yawControl_.lock()) {
    orientationControl->removeListener(this);
  }
  if (auto orientationControl = pitchControl_.lock()) {
    orientationControl->removeListener(this);
  }
  if (auto orientationControl = rollControl_.lock()) {
    orientationControl->removeListener(this);
  }

  /*
  if (auto comboBox = routingComboBox_.lock()) {
    comboBox->removeListener(this);
  }
  if (auto comboBox = colourComboBox_.lock()) {
    comboBox->removeListener(this);
  }
  if (auto slider = gainSlider_.lock()) {
    slider->removeListener(this);
  }
  if (auto slider = azimuthSlider_.lock()) {
    slider->removeListener(this);
  }
  if (auto slider = elevationSlider_.lock()) {
    slider->removeListener(this);
  }
  if (auto slider = distanceSlider_.lock()) {
    slider->removeListener(this);
  }
  if (auto slider = sizeSlider_.lock()) {
    slider->removeListener(this);
  }
  if (auto slider = widthSlider_.lock()) {
    slider->removeListener(this);
  }
  if (auto slider = heightSlider_.lock()) {
    slider->removeListener(this);
  }
  if (auto slider = depthSlider_.lock()) {
    slider->removeListener(this);
  }
  if (auto slider = diffuseSlider_.lock()) {
    slider->removeListener(this);
  }
  if (auto slider = factorSlider_.lock()) {
    slider->removeListener(this);
  }
  if (auto slider = rangeSlider_.lock()) {
    slider->removeListener(this);
  }
  if (auto button = linkSizeButton_.lock()) {
    button->removeListener(this);
  }
  if (auto button = divergenceButton_.lock()) {
    button->removeListener(this);
  }
  if (auto panner = pannerTopView_.lock()) {
    panner->removeListener(this);
  }
  if (auto panner = pannerSideView_.lock()) {
    panner->removeListener(this);
  }
  */
}

void BinauralMonitoringJuceFrontendConnector::setYawView(std::shared_ptr<OrientationView> view)
{
  view->addListener(this);
  yawControl_ = view;
  setYaw(cachedYaw_);
}

void BinauralMonitoringJuceFrontendConnector::setPitchView(std::shared_ptr<OrientationView> view)
{
  view->addListener(this);
  pitchControl_ = view;
  setPitch(cachedPitch_);
}

void BinauralMonitoringJuceFrontendConnector::setRollView(std::shared_ptr<OrientationView> view)
{
  view->addListener(this);
  rollControl_ = view;
  setRoll(cachedRoll_);
}

/*
void BinauralMonitoringJuceFrontendConnector::setNameTextEditor(
    std::shared_ptr<EarNameTextEditor> textEditor) {
  nameTextEditor_ = textEditor;
  setName(cachedName_);
}

void BinauralMonitoringJuceFrontendConnector::setColourComboBox(
    std::shared_ptr<EarComboBox> comboBox) {
  comboBox->addListener(this);
  colourComboBox_ = comboBox;
  setColour(cachedColour_);
}

void BinauralMonitoringJuceFrontendConnector::setPannerTopView(
    std::shared_ptr<PannerTopView> panner) {
  panner->addListener(this);
  pannerTopView_ = panner;
  setColour(cachedColour_);
}

void BinauralMonitoringJuceFrontendConnector::setPannerSideView(
    std::shared_ptr<PannerSideView> panner) {
  panner->addListener(this);
  pannerSideView_ = panner;
  setColour(cachedColour_);
}

void BinauralMonitoringJuceFrontendConnector::setRoutingComboBox(
    std::shared_ptr<EarComboBox> comboBox) {
  comboBox->addListener(this);
  routingComboBox_ = comboBox;
  setRouting(cachedRouting_);
}

void BinauralMonitoringJuceFrontendConnector::setGainSlider(
    std::shared_ptr<EarSlider> slider) {
  slider->addListener(this);
  gainSlider_ = slider;
  setGain(cachedGain_);
}

void BinauralMonitoringJuceFrontendConnector::setAzimuthSlider(
    std::shared_ptr<EarInvertedSlider> slider) {
  slider->addListener(this);
  azimuthSlider_ = slider;
  setAzimuth(cachedAzimuth_);
}

void BinauralMonitoringJuceFrontendConnector::setElevationSlider(
    std::shared_ptr<EarSlider> slider) {
  slider->addListener(this);
  elevationSlider_ = slider;
  setElevation(cachedElevation_);
}

void BinauralMonitoringJuceFrontendConnector::setDistanceSlider(
    std::shared_ptr<EarSlider> slider) {
  slider->addListener(this);
  distanceSlider_ = slider;
  setDistance(cachedDistance_);
}

void BinauralMonitoringJuceFrontendConnector::setLinkSizeButton(
    std::shared_ptr<EarButton> button) {
  button->addListener(this);
  linkSizeButton_ = button;
  setLinkSize(cachedLinkSize_);
}
void BinauralMonitoringJuceFrontendConnector::setSizeLabel(std::shared_ptr<Label> label) {
  sizeLabel_ = label;
  setLinkSize(cachedLinkSize_);
}
void BinauralMonitoringJuceFrontendConnector::setSizeSlider(
    std::shared_ptr<EarSlider> slider) {
  slider->addListener(this);
  sizeSlider_ = slider;
  setSize(cachedSize_);
  setLinkSize(cachedLinkSize_);
}
void BinauralMonitoringJuceFrontendConnector::setWidthLabel(std::shared_ptr<Label> label) {
  widthLabel_ = label;
  setLinkSize(cachedLinkSize_);
}
void BinauralMonitoringJuceFrontendConnector::setWidthSlider(
    std::shared_ptr<EarSlider> slider) {
  slider->addListener(this);
  widthSlider_ = slider;
  setWidth(cachedWidth_);
  setLinkSize(cachedLinkSize_);
}
void BinauralMonitoringJuceFrontendConnector::setHeightLabel(
    std::shared_ptr<Label> label) {
  heightLabel_ = label;
  setLinkSize(cachedLinkSize_);
}
void BinauralMonitoringJuceFrontendConnector::setHeightSlider(
    std::shared_ptr<EarSlider> slider) {
  slider->addListener(this);
  heightSlider_ = slider;
  setHeight(cachedHeight_);
  setLinkSize(cachedLinkSize_);
}
void BinauralMonitoringJuceFrontendConnector::setDepthLabel(std::shared_ptr<Label> label) {
  depthLabel_ = label;
  setLinkSize(cachedLinkSize_);
}
void BinauralMonitoringJuceFrontendConnector::setDepthSlider(
    std::shared_ptr<EarSlider> slider) {
  slider->addListener(this);
  depthSlider_ = slider;
  setDepth(cachedDepth_);
  setLinkSize(cachedLinkSize_);
}
void BinauralMonitoringJuceFrontendConnector::setDiffuseSlider(
    std::shared_ptr<EarSlider> slider) {
  slider->addListener(this);
  diffuseSlider_ = slider;
  setDiffuse(cachedDiffuse_);
}
void BinauralMonitoringJuceFrontendConnector::setDivergenceButton(
    std::shared_ptr<EarButton> button) {
  button->addListener(this);
  divergenceButton_ = button;
  setDivergence(cachedDivergence_);
}
void BinauralMonitoringJuceFrontendConnector::setFactorLabel(
    std::shared_ptr<Label> label) {
  factorLabel_ = label;
  setDivergence(cachedDivergence_);
}
void BinauralMonitoringJuceFrontendConnector::setFactorSlider(
    std::shared_ptr<EarSlider> slider) {
  slider->addListener(this);
  factorSlider_ = slider;
  setFactor(cachedFactor_);
  setDivergence(cachedDivergence_);
}
void BinauralMonitoringJuceFrontendConnector::setRangeLabel(std::shared_ptr<Label> label) {
  rangeLabel_ = label;
  setDivergence(cachedDivergence_);
}
void BinauralMonitoringJuceFrontendConnector::setRangeSlider(
    std::shared_ptr<EarSlider> slider) {
  slider->addListener(this);
  rangeSlider_ = slider;
  setRange(cachedRange_);
  setDivergence(cachedDivergence_);
}
*/

void BinauralMonitoringJuceFrontendConnector::setYaw(float yaw)
{
  if(auto orientationControl = yawControl_.lock()) {
    orientationControl->setValue(yaw, dontSendNotification);
  }
  cachedYaw_ = yaw;
}

void BinauralMonitoringJuceFrontendConnector::setPitch(float pitch)
{
  if(auto orientationControl = pitchControl_.lock()) {
    orientationControl->setValue(pitch, dontSendNotification);
  }
  cachedPitch_ = pitch;
}

void BinauralMonitoringJuceFrontendConnector::setRoll(float roll)
{
  if(auto orientationControl = rollControl_.lock()) {
    orientationControl->setValue(roll, dontSendNotification);
  }
  cachedRoll_ = roll;
}

/*
void BinauralMonitoringJuceFrontendConnector::setName(const std::string& name) {
  if (auto nameTextEditorLocked = nameTextEditor_.lock()) {
    nameTextEditorLocked->setText(name);
  }
  cachedName_ = name;
}

void BinauralMonitoringJuceFrontendConnector::setColour(Colour colour) {
  if (auto colourComboBoxLocked = colourComboBox_.lock()) {
    colourComboBoxLocked->clearEntries();
    colourComboBoxLocked->addColourEntry(colour);
    colourComboBoxLocked->selectEntry(0, dontSendNotification);
  }
  if (auto pannerTopViewLocked = pannerTopView_.lock()) {
    pannerTopViewLocked->setColour(PannerTopView::highlightColourId, colour);
    pannerTopViewLocked->repaint();
  }
  if (auto pannerSideViewLocked = pannerSideView_.lock()) {
    pannerSideViewLocked->setColour(PannerSideView::highlightColourId, colour);
    pannerSideViewLocked->repaint();
  }
  cachedColour_ = colour;
}

void BinauralMonitoringJuceFrontendConnector::setRouting(int routing) {
  if (auto routingComboBoxLocked = routingComboBox_.lock()) {
    routingComboBoxLocked->selectEntry(routing, dontSendNotification);
  }
  cachedRouting_ = routing;
}

void BinauralMonitoringJuceFrontendConnector::setGain(float gain) {
  if (auto gainSliderLocked = gainSlider_.lock()) {
    gainSliderLocked->setValue(gain, dontSendNotification);
  }
  cachedGain_ = gain;
}

void BinauralMonitoringJuceFrontendConnector::setAzimuth(float azimuth) {
  if (auto azimuthSliderLocked = azimuthSlider_.lock()) {
    azimuthSliderLocked->setValue(azimuth, dontSendNotification);
  }
  if (auto pannerTopViewLocked = pannerTopView_.lock()) {
    pannerTopViewLocked->setAzimuth(azimuth, dontSendNotification);
  }
  cachedAzimuth_ = azimuth;
}

void BinauralMonitoringJuceFrontendConnector::setElevation(float elevation) {
  if (auto elevationSliderLocked = elevationSlider_.lock()) {
    elevationSliderLocked->setValue(elevation, dontSendNotification);
  }
  if (auto pannerSideViewLocked = pannerSideView_.lock()) {
    pannerSideViewLocked->setElevation(elevation, dontSendNotification);
  }
  cachedElevation_ = elevation;
}

void BinauralMonitoringJuceFrontendConnector::setDistance(float distance) {
  if (auto distanceSliderLocked = distanceSlider_.lock()) {
    distanceSliderLocked->setValue(distance, dontSendNotification);
  }
  if (auto pannerTopViewLocked = pannerTopView_.lock()) {
    pannerTopViewLocked->setDistance(distance, dontSendNotification);
  }
  cachedDistance_ = distance;
}

void BinauralMonitoringJuceFrontendConnector::setLinkSize(bool linkSize) {
  auto linkSizeButtonLocked = linkSizeButton_.lock();
  auto sizeLabelLocked = sizeLabel_.lock();
  auto sizeSliderLocked = sizeSlider_.lock();
  auto widthLabelLocked = widthLabel_.lock();
  auto widthSliderLocked = widthSlider_.lock();
  auto heightLabelLocked = heightLabel_.lock();
  auto heightSliderLocked = heightSlider_.lock();
  auto depthLabelLocked = depthLabel_.lock();
  auto depthSliderLocked = depthSlider_.lock();
  if (linkSizeButtonLocked && sizeLabelLocked && sizeSliderLocked &&
      widthLabelLocked && widthSliderLocked && heightLabelLocked &&
      heightSliderLocked && depthLabelLocked && depthSliderLocked) {
    linkSizeButtonLocked->setToggleState(linkSize, dontSendNotification);
    if (linkSize) {
      sizeSliderLocked->setGrabFocusOnTextChange(false);
      widthSliderLocked->setGrabFocusOnTextChange(true);
      heightSliderLocked->setGrabFocusOnTextChange(true);
      depthSliderLocked->setGrabFocusOnTextChange(true);
      sizeLabelLocked->setAlpha(Emphasis::disabled);
      sizeSliderLocked->setAlpha(Emphasis::disabled);
      sizeSliderLocked->setEnabled(false);
      widthLabelLocked->setAlpha(1.f);
      widthSliderLocked->setAlpha(1.f);
      widthSliderLocked->setEnabled(true);
      heightLabelLocked->setAlpha(1.f);
      heightSliderLocked->setAlpha(1.f);
      heightSliderLocked->setEnabled(true);
      depthLabelLocked->setAlpha(1.f);
      depthSliderLocked->setAlpha(1.f);
      depthSliderLocked->setEnabled(true);
      widthSliderLocked->grabKeyboardFocus();
    } else {
      widthSliderLocked->setValue(sizeSliderLocked->getValue(),
                                  sendNotificationAsync);
      heightSliderLocked->setValue(sizeSliderLocked->getValue(),
                                   sendNotificationAsync);
      depthSliderLocked->setValue(sizeSliderLocked->getValue(),
                                  sendNotificationAsync);
      sizeSliderLocked->setGrabFocusOnTextChange(true);
      widthSliderLocked->setGrabFocusOnTextChange(false);
      heightSliderLocked->setGrabFocusOnTextChange(false);
      depthSliderLocked->setGrabFocusOnTextChange(false);
      sizeLabelLocked->setAlpha(1.f);
      sizeSliderLocked->setAlpha(1.f);
      sizeSliderLocked->setEnabled(true);
      widthLabelLocked->setAlpha(Emphasis::disabled);
      widthSliderLocked->setAlpha(Emphasis::disabled);
      widthSliderLocked->setEnabled(false);
      heightLabelLocked->setAlpha(Emphasis::disabled);
      heightSliderLocked->setAlpha(Emphasis::disabled);
      heightSliderLocked->setEnabled(false);
      depthLabelLocked->setAlpha(Emphasis::disabled);
      depthSliderLocked->setAlpha(Emphasis::disabled);
      depthSliderLocked->setEnabled(false);
      sizeSliderLocked->grabKeyboardFocus();
    }
  }
  cachedLinkSize_ = linkSize;
}

void BinauralMonitoringJuceFrontendConnector::setSize(float size) {
  if (auto sizeSliderLocked = sizeSlider_.lock()) {
    sizeSliderLocked->setValue(size, dontSendNotification);
  }
  setWidth(size);
  setHeight(size);
  setDepth(size);
  cachedSize_ = size;
}

void BinauralMonitoringJuceFrontendConnector::setWidth(float width) {
  if (auto widthSliderLocked = widthSlider_.lock()) {
    widthSliderLocked->setValue(width, dontSendNotification);
  }
  cachedWidth_ = width;
}

void BinauralMonitoringJuceFrontendConnector::setHeight(float height) {
  if (auto heightSliderLocked = heightSlider_.lock()) {
    heightSliderLocked->setValue(height, dontSendNotification);
  }
  cachedHeight_ = height;
}

void BinauralMonitoringJuceFrontendConnector::setDepth(float depth) {
  if (auto depthSliderLocked = depthSlider_.lock()) {
    depthSliderLocked->setValue(depth, dontSendNotification);
  }
  cachedDepth_ = depth;
}

void BinauralMonitoringJuceFrontendConnector::setDiffuse(float diffuse) {
  if (auto diffuseSliderLocked = diffuseSlider_.lock()) {
    diffuseSliderLocked->setValue(diffuse, dontSendNotification);
  }
  cachedDiffuse_ = diffuse;
}

void BinauralMonitoringJuceFrontendConnector::setDivergence(bool divergence) {
  auto divergenceButtonLocked = divergenceButton_.lock();
  auto factorLabelLocked = factorLabel_.lock();
  auto factorSliderLocked = factorSlider_.lock();
  auto rangeLabelLocked = rangeLabel_.lock();
  auto rangeSliderLocked = rangeSlider_.lock();
  if (divergenceButtonLocked && rangeLabelLocked && rangeSliderLocked) {
    divergenceButtonLocked->setToggleState(divergence, dontSendNotification);
    if (divergence) {
      factorLabelLocked->setAlpha(Emphasis::full);
      factorSliderLocked->setEnabled(true);
      factorSliderLocked->setAlpha(Emphasis::full);
      rangeLabelLocked->setAlpha(Emphasis::full);
      rangeSliderLocked->setEnabled(true);
      rangeSliderLocked->setAlpha(Emphasis::full);
    } else {
      factorLabelLocked->setAlpha(Emphasis::disabled);
      factorSliderLocked->setEnabled(false);
      factorSliderLocked->setAlpha(Emphasis::disabled);
      rangeLabelLocked->setAlpha(Emphasis::disabled);
      rangeSliderLocked->setEnabled(false);
      rangeSliderLocked->setAlpha(Emphasis::disabled);
    }
  }
  cachedDivergence_ = divergence;
}

void BinauralMonitoringJuceFrontendConnector::setFactor(float factor) {
  if (auto factorSliderLocked = factorSlider_.lock()) {
    factorSliderLocked->setValue(factor, dontSendNotification);
  }
  cachedFactor_ = factor;
}

void BinauralMonitoringJuceFrontendConnector::setRange(float range) {
  if (auto rangeSliderLocked = rangeSlider_.lock()) {
    rangeSliderLocked->setValue(range, dontSendNotification);
  }
  cachedRange_ = range;
}
*/
void BinauralMonitoringJuceFrontendConnector::parameterValueChanged(int parameterIndex,
                                                         float newValue) {
  updater_.callOnMessageThread([this, parameterIndex, newValue]() {
    using ParameterId = ui::BinauralMonitoringFrontendBackendConnector::ParameterId;
    switch(parameterIndex) {
      case 0:
        notifyParameterChanged(ParameterId::YAW, p_->getYaw()->get());
        setYaw(p_->getYaw()->get());
        break;
      case 1:
        notifyParameterChanged(ParameterId::PITCH, p_->getPitch()->get());
        setPitch(p_->getPitch()->get());
        break;
      case 2:
        notifyParameterChanged(ParameterId::ROLL, p_->getRoll()->get());
        setRoll(p_->getRoll()->get());
        break;
    }

    /*
    switch (parameterIndex) {
      case 0:
        notifyParameterChanged(ParameterId::ROUTING, p_->getRouting()->get());
        setRouting(p_->getRouting()->get());
        break;
      case 1:
        notifyParameterChanged(ParameterId::GAIN,
                               Decibels::decibelsToGain(p_->getGain()->get()));
        setGain(p_->getGain()->get());
        break;
      case 2:
        notifyParameterChanged(ParameterId::AZIMUTH, p_->getAzimuth()->get());
        setAzimuth(p_->getAzimuth()->get());
        break;
      case 3:
        notifyParameterChanged(ParameterId::ELEVATION, p_->getElevation()->get());
        setElevation(p_->getElevation()->get());
        break;
      case 4:
        notifyParameterChanged(ParameterId::DISTANCE, p_->getDistance()->get());
        setDistance(p_->getDistance()->get());
        break;
      case 5:
        setLinkSize(p_->getLinkSize()->get());
        break;
      case 6:
        setSize(p_->getSize()->get());
        break;
      case 7:
        notifyParameterChanged(ParameterId::WIDTH, p_->getWidth()->get() * 360.f);
        setWidth(p_->getWidth()->get());
        break;
      case 8:
        notifyParameterChanged(ParameterId::HEIGHT,
                               p_->getHeight()->get() * 360.f);
        setHeight(p_->getHeight()->get());
        break;
      case 9:
        notifyParameterChanged(ParameterId::DEPTH, p_->getDepth()->get());
        setDepth(p_->getDepth()->get());
        break;
      case 10:
        notifyParameterChanged(ParameterId::DIFFUSE, p_->getDiffuse()->get());
        setDiffuse(p_->getDiffuse()->get());
        break;
      case 11:
        setDivergence(p_->getDivergence()->get());
        if (p_->getDivergence()->get()) {
          notifyParameterChanged(ParameterId::FACTOR, p_->getFactor()->get());
          notifyParameterChanged(ParameterId::RANGE, p_->getRange()->get());
        } else {
          notifyParameterChanged(ParameterId::FACTOR, 0.f);
          notifyParameterChanged(ParameterId::RANGE, 0.f);
        }
        break;
      case 12:
        notifyParameterChanged(ParameterId::FACTOR, p_->getFactor()->get());
        setFactor(p_->getFactor()->get());
        break;
      case 13:
        notifyParameterChanged(ParameterId::RANGE, p_->getRange()->get());
        setRange(p_->getRange()->get());
        break;
    }
    */
  });
}

void BinauralMonitoringJuceFrontendConnector::orientationValueChanged(ear::plugin::ui::OrientationView * view)
{
  if(!yawControl_.expired() && view == yawControl_.lock().get()) {
    *(p_->getYaw()) = view->getValue();

  } else if(!pitchControl_.expired() && view == pitchControl_.lock().get()) {
    *(p_->getPitch()) = view->getValue();

  } else if(!rollControl_.expired() && view == rollControl_.lock().get()) {
    *(p_->getRoll()) = view->getValue();
  }
}

void BinauralMonitoringJuceFrontendConnector::orientationDragStarted(ear::plugin::ui::OrientationView * view)
{
  if(!yawControl_.expired() && view == yawControl_.lock().get()) {
    p_->getYaw()->beginChangeGesture();

  } else if(!pitchControl_.expired() && view == pitchControl_.lock().get()) {
    p_->getPitch()->beginChangeGesture();

  } else if(!rollControl_.expired() && view == rollControl_.lock().get()) {
    p_->getRoll()->beginChangeGesture();
  }
}

void BinauralMonitoringJuceFrontendConnector::orientationDragEnded(ear::plugin::ui::OrientationView * view)
{
  if(!yawControl_.expired() && view == yawControl_.lock().get()) {
    p_->getYaw()->endChangeGesture();

  } else if(!pitchControl_.expired() && view == pitchControl_.lock().get()) {
    p_->getPitch()->endChangeGesture();

  } else if(!rollControl_.expired() && view == rollControl_.lock().get()) {
    p_->getRoll()->endChangeGesture();
  }
}

/*
void BinauralMonitoringJuceFrontendConnector::sliderValueChanged(Slider* slider) {
  if (!gainSlider_.expired() && slider == gainSlider_.lock().get()) {
    *(p_->getGain()) = slider->getValue();
  } else if (!azimuthSlider_.expired() &&
             slider == azimuthSlider_.lock().get()) {
    *(p_->getAzimuth()) = slider->getValue();
  } else if (!elevationSlider_.expired() &&
             slider == elevationSlider_.lock().get()) {
    *(p_->getElevation()) = slider->getValue();
  } else if (!distanceSlider_.expired() &&
             slider == distanceSlider_.lock().get()) {
    *(p_->getDistance()) = slider->getValue();
  } else if (!sizeSlider_.expired() && slider == sizeSlider_.lock().get()) {
    *(p_->getWidth()) = slider->getValue();
    *(p_->getHeight()) = slider->getValue();
    *(p_->getDepth()) = slider->getValue();
  } else if (!widthSlider_.expired() && slider == widthSlider_.lock().get()) {
    *(p_->getWidth()) = slider->getValue();
  } else if (!heightSlider_.expired() && slider == heightSlider_.lock().get()) {
    *(p_->getHeight()) = slider->getValue();
  } else if (!diffuseSlider_.expired() &&
             slider == diffuseSlider_.lock().get()) {
    *(p_->getDiffuse()) = slider->getValue();
  } else if (!factorSlider_.expired() && slider == factorSlider_.lock().get()) {
    *(p_->getFactor()) = slider->getValue();
  } else if (!rangeSlider_.expired() && slider == rangeSlider_.lock().get()) {
    *(p_->getRange()) = slider->getValue();
  }
}

void BinauralMonitoringJuceFrontendConnector::sliderDragStarted(Slider* slider) {
  if (!gainSlider_.expired() && slider == gainSlider_.lock().get()) {
    p_->getGain()->beginChangeGesture();
  } else if (!azimuthSlider_.expired() &&
             slider == azimuthSlider_.lock().get()) {
    p_->getAzimuth()->beginChangeGesture();
  } else if (!elevationSlider_.expired() &&
             slider == elevationSlider_.lock().get()) {
    p_->getElevation()->beginChangeGesture();
  } else if (!distanceSlider_.expired() &&
             slider == distanceSlider_.lock().get()) {
    p_->getDistance()->beginChangeGesture();
  } else if (!sizeSlider_.expired() && slider == sizeSlider_.lock().get()) {
    p_->getWidth()->beginChangeGesture();
    p_->getHeight()->beginChangeGesture();
    p_->getDepth()->beginChangeGesture();
  } else if (!widthSlider_.expired() && slider == widthSlider_.lock().get()) {
    p_->getWidth()->beginChangeGesture();
  } else if (!heightSlider_.expired() && slider == heightSlider_.lock().get()) {
    p_->getHeight()->beginChangeGesture();
  } else if (!diffuseSlider_.expired() &&
             slider == diffuseSlider_.lock().get()) {
    p_->getDiffuse()->beginChangeGesture();
  } else if (!factorSlider_.expired() && slider == factorSlider_.lock().get()) {
    p_->getFactor()->beginChangeGesture();
  } else if (!rangeSlider_.expired() && slider == rangeSlider_.lock().get()) {
    p_->getRange()->beginChangeGesture();
  }
}

void BinauralMonitoringJuceFrontendConnector::sliderDragEnded(Slider* slider) {
  if (!gainSlider_.expired() && slider == gainSlider_.lock().get()) {
    p_->getGain()->endChangeGesture();
  } else if (!azimuthSlider_.expired() &&
             slider == azimuthSlider_.lock().get()) {
    p_->getAzimuth()->endChangeGesture();
  } else if (!elevationSlider_.expired() &&
             slider == elevationSlider_.lock().get()) {
    p_->getElevation()->endChangeGesture();
  } else if (!distanceSlider_.expired() &&
             slider == distanceSlider_.lock().get()) {
    p_->getDistance()->endChangeGesture();
  } else if (!sizeSlider_.expired() && slider == sizeSlider_.lock().get()) {
    p_->getWidth()->endChangeGesture();
    p_->getHeight()->endChangeGesture();
    p_->getDepth()->endChangeGesture();
  } else if (!widthSlider_.expired() && slider == widthSlider_.lock().get()) {
    p_->getWidth()->endChangeGesture();
  } else if (!heightSlider_.expired() && slider == heightSlider_.lock().get()) {
    p_->getHeight()->endChangeGesture();
  } else if (!diffuseSlider_.expired() &&
             slider == diffuseSlider_.lock().get()) {
    p_->getDiffuse()->endChangeGesture();
  } else if (!factorSlider_.expired() && slider == factorSlider_.lock().get()) {
    p_->getFactor()->endChangeGesture();
  } else if (!rangeSlider_.expired() && slider == rangeSlider_.lock().get()) {
    p_->getRange()->endChangeGesture();
  }
}

void BinauralMonitoringJuceFrontendConnector::pannerValueChanged(
    ear::plugin::ui::PannerTopView* panner) {
  if (!pannerTopView_.expired() && panner == pannerTopView_.lock().get()) {
    *(p_->getAzimuth()) = panner->getAzimuth();
    *(p_->getDistance()) = panner->getDistance();
  }
}

void BinauralMonitoringJuceFrontendConnector::pannerDragStarted(
    ear::plugin::ui::PannerTopView* panner) {
  if (!pannerTopView_.expired() && panner == pannerTopView_.lock().get()) {
    p_->getAzimuth()->beginChangeGesture();
    p_->getDistance()->beginChangeGesture();
  }
}

void BinauralMonitoringJuceFrontendConnector::pannerDragEnded(
    ear::plugin::ui::PannerTopView* panner) {
  if (!pannerTopView_.expired() && panner == pannerTopView_.lock().get()) {
    p_->getAzimuth()->endChangeGesture();
    p_->getDistance()->endChangeGesture();
  }
}

void BinauralMonitoringJuceFrontendConnector::pannerValueChanged(
    ear::plugin::ui::PannerSideView* panner) {
  if (!pannerSideView_.expired() && panner == pannerSideView_.lock().get()) {
    *(p_->getElevation()) = panner->getElevation();
  }
}

void BinauralMonitoringJuceFrontendConnector::pannerDragStarted(
    ear::plugin::ui::PannerSideView* panner) {
  if (!pannerSideView_.expired() && panner == pannerSideView_.lock().get()) {
    p_->getElevation()->beginChangeGesture();
  }
}

void BinauralMonitoringJuceFrontendConnector::pannerDragEnded(
    ear::plugin::ui::PannerSideView* panner) {
  if (!pannerSideView_.expired() && panner == pannerSideView_.lock().get()) {
    p_->getElevation()->endChangeGesture();
  }
}

void BinauralMonitoringJuceFrontendConnector::comboBoxChanged(EarComboBox* comboBox) {
  if (!routingComboBox_.expired() &&
      comboBox == routingComboBox_.lock().get()) {
    *(p_->getRouting()) = comboBox->getSelectedEntryIndex();
  }
}

void BinauralMonitoringJuceFrontendConnector::buttonClicked(Button* button) {
  if (!divergenceButton_.expired() &&
      button == divergenceButton_.lock().get()) {
    // note: getToggleState still has the old value when this is called
    *(p_->getDivergence()) = !button->getToggleState();
  } else if (!linkSizeButton_.expired() &&
             button == linkSizeButton_.lock().get()) {
    *(p_->getLinkSize()) = !button->getToggleState();
  }
}
*/

}  // namespace ui
}  // namespace plugin
}  // namespace ear
