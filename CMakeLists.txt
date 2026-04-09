# Root CMakeLists.txt for Toucan keyboard ZMK module.
# Compiles custom input processors when the pointing feature is enabled.

if(CONFIG_ZMK_POINTING)
  target_sources(app PRIVATE src/circular_scroll.c)
endif()
