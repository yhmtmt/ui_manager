add_library(ui_manager SHARED f_ui_manager.cpp)

target_include_directories(ui_manager PUBLIC ${PROJECT_SOURCE_DIR}/include)
install(TARGETS ui_manager DESTINATION lib)
