######################################################################################
# Packages needed for examples
######################################################################################

add_executable(glview glview.c)
add_executable(cvview cvview.c)

# Mac just has everything already
if(APPLE)
  set(CMAKE_EXE_LINKER_FLAGS "-framework OpenGL -framework GLUT")
  #find_package(OpenCv REQUIRED)
  include_directories(/opt/local/include/opencv)
  include_directories(${LIBUSB_1_INCLUDE_DIRS})  
  target_link_libraries(glview freenect)
  target_link_libraries(cvview freenect /opt/local/lib/libcxcore.dylib /opt/local/lib/libcvaux.dylib /opt/local/lib/libcv.dylib /opt/local/lib/libhighgui.dylib /opt/local/lib/libml.dylib)
# Linux, not so much
else()
  find_package(Threads REQUIRED)
  find_package(OpenGL REQUIRED)
  find_package(GLUT REQUIRED)
  include_directories(${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS} ${USB_INCLUDE_DIRS})
  target_link_libraries(glview freenect ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${CMAKE_THREAD_LIBS_INIT} m)
endif()

install (TARGETS glview
  DESTINATION bin)
