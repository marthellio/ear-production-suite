set(SOURCES_HOA_INPUT
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/binary_data.cpp

	# TODO - C+P'd from DS plugin - remove unnecessary components once UI dev finished
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/components/look_and_feel/slider.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/components/look_and_feel/name_text_editor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/components/ear_combo_box.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/components/ear_slider_label.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/components/level_meter_calculator.cpp
	
    src/hoa_frontend_connector.cpp
    src/hoa_plugin_editor.cpp
    src/hoa_plugin_processor.cpp
)

# Input plugin code suffixes to start from 10
add_juce_vst3_plugin(hoa
  SOURCES ${SOURCES_HOA_INPUT}
  CODE_SUFFIX "12"
  DISPLAY_NAME "EAR HOA" 
  DESCRIPTION "The metadata input plugin for scene-based audio"
  OUTPUT_NAME "EAR HOA")
  
target_link_libraries(hoa_VST3 PRIVATE ear-plugin-base)

install_juce_vst3_plugin(hoa "${EPS_PLUGIN_INSTALL_PREFIX}VST3/ear-production-suite")
