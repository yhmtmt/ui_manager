find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 REQUIRED)
#find_package(glm REQUIRED)
#Filter message's flatbuffer definition
file(GLOB SAMPLE_FLATS *.fbs)
set(SAMPLE_FLAT_SRC_DIR ./)


flatc_generate_cpp(UI_MANAGER_FLAT_HDRS UI_MANAGER_FLAT_BFBS ${CMAKE_CURRENT_SOURCE_DIR}  ui_manager_msg.fbs)
install(FILES ${UI_MANAGER_FLAT_BFBS}
  PERMISSIONS OWNER_EXECUTE OWNER_READ OWNER_WRITE
  DESTINATION lib)

file(GLOB UTILS ${CMAKE_CURRENT_SOURCE_DIR}/util/*.cpp)
file(GLOB RES ${CMAKE_CURRENT_SOURCE_DIR}/res/*)
add_library(ui_manager SHARED f_ui_manager.cpp f_glfw_window.cpp ui_command.cpp ${CHANS} ${UTILS} ${UI_MANAGER_FLAT_HDRS})

target_include_directories(ui_manager PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_include_directories(ui_manager PUBLIC ${PROJECT_SOURCE_DIR}/channels)
target_include_directories(ui_manager PUBLIC ${OpenCV_INCLUDE_DIRS})
target_include_directories(ui_manager PUBLIC ${OPENGL_INCLUDE_DIRS})
target_include_directories(ui_manager PUBLIC ${GLUT_INCLUDE_DIRS})
target_include_directories(ui_manager PUBLIC ${GLEW_INCLUDE_DIR})
target_link_libraries(ui_manager PUBLIC ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${GLEW_LIBRARIES} glfw )

install(TARGETS ui_manager DESTINATION lib)
install(FILES ${RES} DESTINATION lib)
