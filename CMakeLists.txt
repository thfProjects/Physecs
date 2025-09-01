cmake_minimum_required(VERSION "3.19.2")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

project(Physecs)

include(FetchContent)

option(BUILD_DEMO "Build demo")
option(PROFILE "Profile")

set(PHYSECS_LIB Physecs)

file(GLOB_RECURSE LIB_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

add_library("${PHYSECS_LIB}" SHARED)

set_property(TARGET "${PHYSECS_LIB}" PROPERTY CXX_STANDARD 17)

set_property(TARGET "${PHYSECS_LIB}" PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded")

target_sources("${PHYSECS_LIB}" PRIVATE ${LIB_SOURCES})

target_include_directories("${PHYSECS_LIB}" PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_include_directories("${PHYSECS_LIB}" PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/Physecs")
target_include_directories("${PHYSECS_LIB}" PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/Physecs/Joints")
target_include_directories("${PHYSECS_LIB}" PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/vendor/glm 0.9.9.8")
target_include_directories("${PHYSECS_LIB}" PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/vendor/entt-3.12.2/single_include/entt")

add_definitions(
		-DGLM_FORCE_INLINE
		-DPHYSECS_EXPORTS
)

if (PROFILE)
	FetchContent_Declare (
		tracy
		GIT_REPOSITORY https://github.com/wolfpld/tracy.git
		GIT_TAG v0.12.2
		GIT_SHALLOW TRUE
		GIT_PROGRESS TRUE
	)

	FetchContent_MakeAvailable(tracy)

	set_property(TARGET TracyClient PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded")

	target_link_libraries("${PHYSECS_LIB}" PUBLIC TracyClient)

	add_definitions(
		-DTRACY_ENABLE
		-DTRACY_NO_SAMPLING
		-DPHYSECS_PROFILE
	)
endif ()

if (BUILD_DEMO)
	FetchContent_Declare(
		SDL
		GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
		GIT_TAG release-2.32.6
	)

	FetchContent_MakeAvailable(SDL)

	set(PHYSECS_DEMO PhysecsDemo)

	file(GLOB_RECURSE DEMO_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/demo/*.cpp")

	add_executable("${PHYSECS_DEMO}")

	set_property(TARGET "${PHYSECS_DEMO}" PROPERTY CXX_STANDARD 17)

	set_property(TARGET "${PHYSECS_DEMO}" PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded")

	set(GLAD_SRC "${CMAKE_CURRENT_SOURCE_DIR}/vendor/glad3.3core/src/glad.c")

	target_sources("${PHYSECS_DEMO}" PRIVATE ${DEMO_SOURCES} ${GLAD_SRC})

	target_include_directories("${PHYSECS_DEMO}" PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/demo")
	target_include_directories("${PHYSECS_DEMO}" PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/demo/Renderer")
	target_include_directories("${PHYSECS_DEMO}" PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
	target_include_directories("${PHYSECS_DEMO}" PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/vendor/glm 0.9.9.8")
	target_include_directories("${PHYSECS_DEMO}" PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/vendor/entt-3.12.2/single_include/entt")
	target_include_directories("${PHYSECS_DEMO}" PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/vendor/glad3.3core/include")

	target_link_libraries("${PHYSECS_DEMO}" PRIVATE "${PHYSECS_LIB}" SDL2::SDL2 SDL2::SDL2main)
	if (MSVC)
		target_link_libraries("${PHYSECS_DEMO}" PRIVATE opengl32)
	endif ()

	add_definitions(
		-DTINYGLTF_IMPLEMENTATION
		-DTINYGLTF_NO_STB_IMAGE_WRITE
	)

	add_custom_command(TARGET "${PHYSECS_DEMO}" POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_directory_if_different
		${CMAKE_CURRENT_SOURCE_DIR}/res
		${CMAKE_CURRENT_BINARY_DIR}
	)
endif ()
