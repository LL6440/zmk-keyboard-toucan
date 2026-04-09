if(CONFIG_ZMK_INPUT_PROCESSOR_ABSOLUTE_REL_POINTER OR CONFIG_ZMK_INPUT_PROCESSOR_CIRCULAR_SCROLL OR CONFIG_ZMK_INPUT_PROCESSOR_TOUCH_BUTTON)
  zephyr_library()
  zephyr_library_include_directories(${CMAKE_SOURCE_DIR}/include)
endif()

if(CONFIG_ZMK_INPUT_PROCESSOR_ABSOLUTE_REL_POINTER)
  zephyr_library_sources(src/input_processor_absolute_rel_pointer.c)
endif()

if(CONFIG_ZMK_INPUT_PROCESSOR_CIRCULAR_SCROLL)
  zephyr_library_sources(src/input_processor_circular_scroll.c)
endif()

if(CONFIG_ZMK_INPUT_PROCESSOR_TOUCH_BUTTON)
  zephyr_library_sources(src/input_processor_touch_button.c)
endif()
