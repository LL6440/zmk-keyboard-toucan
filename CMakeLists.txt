if(CONFIG_ZMK_INPUT_PROCESSOR_ABSOLUTE_REL_POINTER OR CONFIG_ZMK_INPUT_PROCESSOR_EDGE_SCROLL)
  zephyr_library()
endif()

if(CONFIG_ZMK_INPUT_PROCESSOR_ABSOLUTE_REL_POINTER)
  zephyr_library_sources(src/input_processor_absolute_rel_pointer.c)
endif()

if(CONFIG_ZMK_INPUT_PROCESSOR_EDGE_SCROLL)
  zephyr_library_sources(src/input_processor_edge_scroll.c)
endif()
