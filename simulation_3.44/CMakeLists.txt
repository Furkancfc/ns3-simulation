cmake_minimum_required(VERSION 3.10)
project(simulation_3.44)
include(ExternalProject)


set(dir ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_ROOT ${dir})
set(CMAKE_BUILD_RPATH ${dir}/build)
set(CMAKE_BINARY_DIR  ${dir}/build)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(EXECUTABLE_OUTPUT_NAME "simulation_4")



# NS-3 version configuration
set(NS3_VER "3.44")  # or "3.44"
set(PARALLEL_JOB 6)

function(setup_ns3_directory version)
	set(NS3_DIR "/mnt/nvme0n1p6/Klasorler/Dersler/BitirmeProje/simulation_4/ns-${version}")
	set(NS3_INSTALL_DIR "${NS3_DIR}/install")
	set(NS3_DIR "${NS3_DIR}" PARENT_SCOPE)
	set(NS3_INSTALL_DIR "${NS3_INSTALL_DIR}" PARENT_SCOPE)
endfunction()

setup_ns3_directory(${NS3_VER})

# Common log redirection setup
set(LOG_DIR "${CMAKE_SOURCE_DIR}/logs")
file(MAKE_DIRECTORY ${LOG_DIR})

if(${NS3_VER} STREQUAL "3.27")
	# NS-3.27 specific build with waf
	set(CMAKE_C_COMPILER "/usr/bin/gcc-10")
	set(CMAKE_CXX_COMPILER "/usr/bin/g++-10")
	set(CMAKE_CXX_STANDARD 11)
	
	ExternalProject_Add(
		ns3_project
		SOURCE_DIR "${NS3_DIR}"
		CONFIGURE_COMMAND 
			cd ${NS3_DIR} && 
			CC=${CMAKE_C_COMPILER} CXX=${CMAKE_CXX_COMPILER} 
			python2 waf configure --prefix=${NS3_INSTALL_DIR} 
				> ${LOG_DIR}/ns3_configure.log 2>&1
		BUILD_COMMAND 
			cd ${NS3_DIR} && 
			python2 waf build -j ${PARALLEL_JOB} 
				> ${LOG_DIR}/ns3_build.log 2>&1
		INSTALL_COMMAND 
			cd ${NS3_DIR} && 
			python2 waf install 
				> ${LOG_DIR}/ns3_install.log 2>&1
		BUILD_BYPRODUCTS ${NS3_INSTALL_DIR}/lib/libns3-core.so
	)
elseif(${NS3_VER} STREQUAL "3.44")
	# NS-3.44 specific build with CMake
	set(CMAKE_C_COMPILER "/usr/bin/gcc-13")
	set(CMAKE_CXX_COMPILER "/usr/bin/g++-13")
	set(CMAKE_CXX_STANDARD 20)
	
	ExternalProject_Add(
		ns3_project
		SOURCE_DIR "${NS3_DIR}"
		CONFIGURE_COMMAND 
			cd ${NS3_DIR} && 
			${CMAKE_COMMAND} -DCMAKE_INSTALL_PREFIX=${NS3_INSTALL_DIR} . 
				> ${LOG_DIR}/ns3_configure.log 2>&1
		BUILD_COMMAND 
			cd ${NS3_DIR} && 
			make -j ${PARALLEL_JOB} 
				> ${LOG_DIR}/ns3_build.log 2>&1
		INSTALL_COMMAND 
			cd ${NS3_DIR} && 
			make install -j ${PARALLEL_JOB} 
				> ${LOG_DIR}/ns3_install.log 2>&1
		BUILD_BYPRODUCTS ${NS3_INSTALL_DIR}/lib/libns3-core.so
	)
endif()

# Your simulation executable setup
set(EXEC_SOURCE "${dir}/main.cc")
file(GLOB SRC_FILES "src/*.cc")
file(GLOB NS3_LIBS "${NS3_INSTALL_DIR}/lib/libns3*.so")
add_executable(
  simulation_4 
  ${EXEC_SOURCE} 
  ${SRC_FILES}
)


# Include directories
set(NS3_INSTALL_DIR "${NS3_DIR}/install")
set(INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include")
include_directories(
	${NS3_INSTALL_DIR}
	${INCLUDE_DIRECTORIES}
)
# Link NS-3 libraries
if( ${NS3_VER} EQUAL "3.27")
include_directories(${NS3_INSTALL_DIR}/include/ns3.27)
else()
include_directories(${NS3_INSTALL_DIR}/include/)
endif()
target_link_libraries(simulation_4 ${NS3_LIBS})

set_target_properties(
  simulation_4
  PROPERTIES
  OUTPUT_NAME ${EXECUTABLE_OUTPUT_NAME}
)
# Add custom commands

set(TARGET_ENV_VARS "")
list(APPEND TARGET_ENV_VARS "PROJECT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\"")
list(APPEND TARGET_ENV_VARS "NS3_VERSION=\"${NS3_VER}\"")
list(APPEND TARGET_ENV_VARS "NS3_DIR=\"${NS3_DIR}\"")
list(APPEND TARGET_ENV_VARS "PARALLEL_JOBS=\"${PARALLEL_JOB}\"")
list(APPEND TARGET_ENV_VARS "LOG_DIR=\"${LOG_DIR}\"")
list(APPEND TARGET_ENV_VARS "EXEC_SOURCE=\"${EXEC_SOURCE}\"")
string(REPLACE ";" " " ENV_VARS_STRING "${TARGET_ENV_VARS}")

set(RUN_COMMAND 	"${ENV_VARS_STRING} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${EXECUTABLE_OUTPUT_NAME} > ${LOG_DIR}/output.log 2>&1 "
)
add_custom_target(launch
	COMMAND ${CMAKE_COMMAND} -E make_directory ${LOG_DIR}
	COMMAND ${CMAKE_COMMAND} -E echo "Launching simulation with env vars: ${ENV_VARS_STRING}"
	COMMAND bash -c ${RUN_COMMAND}
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	COMMENT "✅ Launching simulation and writing logs to ${LOG_DIR}/output.log"
	VERBATIM
)
add_custom_target(env_vars_help
	COMMAND ${CMAKE_COMMAND} -E echo "Environment variables for simulation:"
	COMMAND ${CMAKE_COMMAND} -E echo "  ${ENV_VARS_STRING}"
	COMMENT "Display available environment variables for configuration"
)
add_custom_command(
    OUTPUT ${CMAKE_BINARY_DIR}/${FILENAME}
    COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_SOURCE_DIR}/real_folder ${CMAKE_BINARY_DIR}/your_symlink
    COMMENT "Creating symbolic link: your_symlink → real_folder"
)
add_custom_command(
    TARGET simulation_4
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo "🔍 [POST_BUILD] Linking *.copy.* into output dir..."
    COMMAND bash -c "CMAKE_CURRENT_SOURCE_DIR='${CMAKE_CURRENT_SOURCE_DIR}' OUTPUT_DIR='${CMAKE_RUNTIME_OUTPUT_DIRECTORY}' bash ${CMAKE_CURRENT_SOURCE_DIR}/copy.sh"
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "🧩 [POST_BUILD] Running copy.sh to symlink files"
    VERBATIM
)

target_compile_definitions(simulation_4 PRIVATE NS3_LOG_ENABLE)
target_compile_options(simulation_4 PRIVATE -g -O0)  # Debug symbols, no optimization
