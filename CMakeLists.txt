cmake_minimum_required(VERSION 3.10)
project(simulation_4)

# Set the C++ standard to C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(NS3_DIR "/mnt/nvme0n1p6/Klasorler/Dersler/BitirmeProje/ns-3-dev")
# Set the path to the ns-3 installation
set(NS3_INSTALL_DIR "${NS3_DIR}/install")

# Add ExternalProject module
include(ExternalProject)

# Define ns-3 as an external project
ExternalProject_Add(
  ns3_project
  SOURCE_DIR "${NS3_DIR}"
  BINARY_DIR "${NS3_DIR}/build"
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${NS3_INSTALL_DIR}
  BUILD_COMMAND make
  INSTALL_COMMAND make install -j 6
)
add_custom_target(
  build_only_this
  COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target simulation_4 -j 6
  DEPENDS ns3_project
)
# Find the ns-3 package
find_package(ns3 REQUIRED PATHS ${NS3_INSTALL_DIR} NO_DEFAULT_PATH)
# Add all .cc files from the src directory
file(GLOB SRC_FILES src/*.cc)
# Add your executable with all .cc files
add_executable(simulation_4 ./main.cc)

# Add your executable
# Include ns-3 headers
target_include_directories(simulation_4 PRIVATE ${NS3_INSTALL_DIR}/include)
target_include_directories(simulation_4 PRIVATE include/)
target_sources(simulation_4 PRIVATE ${SRC_FILES})
target_link_directories(simulation_4 PRIVATE ${NS3_INSTALL_DIR}/lib)
target_link_directories(simulation_4 PRIVATE ${NS3_INSTALL_DIR}/lib64)
target_link_directories(simulation_4 PRIVATE build/)
# Link against ns-3 libraries
target_link_libraries(simulation_4
  ns3::core
  ns3::network
  ns3::internet
  ns3::wifi
  ns3::propagation
  ns3::mobility
  ns3::applications
  ns3::point-to-point
  ns3::csma
  ns3::olsr
  ns3::aodv
  ns3::dsdv
  ns3::dsr
  ns3::energy
  ns3::flow-monitor
  ns3::uan
  ns3::virtual-net-device
  ns3::fd-net-device
  ns3::lte
  ns3::mesh
  ns3::sixlowpan
  ns3::nix-vector-routing
  ns3::tap-bridge
  ns3::topology-read
  ns3::traffic-control
  ns3::spectrum
  ns3::stats
)

# Add a dependency to ensure ns3 is built before simulation_4
add_dependencies(simulation_4 ns3_project)