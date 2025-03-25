cmake_minimum_required(VERSION 3.10)
project(simulation_3.27)
include(ExternalProject)
set(dir ${CMAKE_CURRENT_SOURCE_DIR})
message(STATUS ${dir})
set(EXECUTABLE_OUTPUT_PATH ${dir} CACHE PATH "Build directory" FORCE)
set(LIBRARY_OUTPUT_PATH ${dir} CACHE PATH "Build directory" FORCE)
message(STATUS ${EXECUTABLE_OUTPUT_PATH})
message(STATUS ${LIBRARY_OUTPUT_PATH})

set(NS3_VER "3.27")
set(PARALLEL_JOB 6)

function(setup_ns3_directory version)
  # Set variables in the current scope
  set(NS3_DIR "/mnt/nvme0n1p6/Klasorler/Dersler/BitirmeProje/simulation_4/ns-${version}")
  set(NS3_INSTALL_DIR "${NS3_DIR}/install")
  set(CMAKE_INSTALL_PREFIX "${NS3_INSTALL_DIR}")
  set(CMAKE_INSTALL_LIBDIR "${NS3_INSTALL_DIR}/lib")

  # Set variables in the parent scope
  set(NS3_DIR "${NS3_DIR}" PARENT_SCOPE)
  set(NS3_INSTALL_DIR "${NS3_INSTALL_DIR}" PARENT_SCOPE)
  set(CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}" PARENT_SCOPE)
  set(CMAKE_INSTALL_LIBDIR "${CMAKE_INSTALL_LIBDIR}" PARENT_SCOPE)
endfunction()

setup_ns3_directory(${NS3_VER})
set(NS3_LIBS "${NS3_INSTALL_DIR}/lib/libns*")
if(NS3_LIBS)
  message(STATUS "NS3 is already built. Skipping build step.")
  set(NS3_ALREADY_BUILT TRUE)
else()
  message(STATUS "NS3 is not found. Will build NS3.")
  set(NS3_ALREADY_BUILT FALSE)
endif()

  if(${NS3_VER} STREQUAL "3.27")
    set(CMAKE_C_STANDARD 10) # Change to 99, 17, or another version as needed
    set(CMAKE_C_STANDARD_REQUIRED ON)
    set(CXXFLAGS ${CXXFLAGS} -Wall -Werror)
    set(CC=/usr/bin/gcc-10)
    set(CXX=/usr/bin/g++-10)
    message(STATUS "NS3_DIR : ${NS3_DIR}")
    message(STATUS "NS3_INSTALL_DIR : ${NS3_INSTALL_DIR}")
    ExternalProject_Add(
      ns3_project
      SOURCE_DIR "${NS3_DIR}"
      CONFIGURE_COMMAND 
          cd ${NS3_DIR} && 
          CC=/usr/bin/gcc-10 CXX=/usr/bin/g++-10 
          python2 waf configure --prefix=${NS3_INSTALL_DIR} 
              > ${CMAKE_SOURCE_DIR}/ns3_configure.log 
      BUILD_COMMAND 
          cd ${NS3_DIR} && 
          python2 waf build -j ${PARALLEL_JOB} 
              > ${CMAKE_SOURCE_DIR}/ns3_build.log
      INSTALL_COMMAND 
          cd ${NS3_DIR} && 
          python2 waf install 
              > ${CMAKE_SOURCE_DIR}/ns3_install.log
      BUILD_BYPRODUCTS ${NS3_INSTALL_DIR}/lib/libns3-core.so
  )
  elseif(${NS3_VER} STREQUAL "3.44")
    set(CC /usr/bin/gcc-13)
    set(CXX /usr/bin/g++-13)
    message(STATUS "NS3_DIR : ${NS3_DIR}")
    message(STATUS "NS3_INSTALL_DIR : ${NS3_INSTALL_DIR}")
    ExternalProject_Add(
      ns3_project
      SOURCE_DIR "${NS3_DIR}"
      CONFIGURE_COMMAND cd ${NS3_DIR} && ${CMAKE_COMMAND} -DCMAKE_INSTALL_PREFIX=${NS3_INSTALL_DIR} .
      BUILD_COMMAND cd ${NS3_DIR} && make . -j ${PARALLEL_JOB}
      INSTALL_COMMAND cd ${NS3_DIR} && make install -j ${PARALLEL_JOB}
      BUILD_BYPRODUCTS ${NS3_INSTALL_DIR}/lib/libns3-core.so
    )
  else()
    set(CC /usr/bin/gcc-13)
    set(CXX /usr/bin/g++-13)
    message(STATUS "NS3_DIR : ${NS3_DIR}")
    message(STATUS "NS3_INSTALL_DIR : ${NS3_INSTALL_DIR}")
    ExternalProject_Add(
      ns3_project
      SOURCE_DIR "${NS3_DIR}"
      CONFIGURE_COMMAND cd ${NS3_DIR} && ${CMAKE_COMMAND} -DCMAKE_INSTALL_PREFIX=${NS3_INSTALL_DIR} . # Add '+' before 'make' here
      BUILD_COMMAND cd ${NS3_DIR} && make build -j ${PARALLEL_JOB} # Add '+' before 'make' here
      INSTALL_COMMAND cd ${NS3_DIR} && make install -j ${PARALLEL_JOB}
      BUILD_BYPRODUCTS ${NS3_INSTALL_DIR}/lib/libns3-core.so
    )
    # Add a custom command to install with sudo (without passing --enable-sudo in CMake)
  endif()

if(NOT NS3_ALREADY_BUILT)
  add_dependencies(simulation_4 ns3_project)
endif()

message(STATUS "Building Simulation")
message(STATUS "NS3_DIR : ${NS3_DIR}")
message(STATUS "NS3_INSTALL_DIR : ${NS3_INSTALL_DIR}")
file(GLOB SRC_FILES src/*.cc)
file(GLOB NS3_LIBS ${NS3_INSTALL_DIR}/lib/*.so)

add_executable(simulation_4 ./main.cc ${SRC_FILES})

target_link_libraries(simulation_4 ${NS3_LIBS})
if( ${NS3_VER} EQUAL "3.27")
include_directories(${NS3_INSTALL_DIR}/include/ns3.27)
else()
include_directories(${NS3_INSTALL_DIR}/include/)
endif()
include_directories(include/)
set(NS3_LIBS "${NS3_INSTALL_DIR}/lib/libns${NS3_VER}*")

# Copy *.copy files to the build path
file(GLOB COPY_FILES "*.copy*")
foreach(COPY_FILE ${COPY_FILES})
get_filename_component(COPY_FILE_NAME ${COPY_FILE} NAME)

  add_custom_command(
    TARGET simulation_4 PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E create_symlink ${COPY_FILE} ${CMAKE_BINARY_DIR}/${COPY_FILE_NAME}
    COMMENT "Creating symbolic link for ${COPY_FILE} in build directory"
  )
endforeach()