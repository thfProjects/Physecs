cmake_minimum_required(VERSION "3.19.2")

project(Physecs)

file(GLOB_RECURSE MY_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

add_library("${CMAKE_PROJECT_NAME}" SHARED)

set_property(TARGET "${CMAKE_PROJECT_NAME}" PROPERTY CXX_STANDARD 17)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

set_property(TARGET "${CMAKE_PROJECT_NAME}" PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded")

set(GLAD_SRC "${CMAKE_CURRENT_SOURCE_DIR}/vendor/glad3.3core/src/glad.c")

target_sources("${CMAKE_PROJECT_NAME}" PRIVATE ${MY_SOURCES} ${GLAD_SRC})

target_include_directories("${CMAKE_PROJECT_NAME}" PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_include_directories("${CMAKE_PROJECT_NAME}" PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_include_directories("${CMAKE_PROJECT_NAME}" PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/Joints")
target_include_directories("${CMAKE_PROJECT_NAME}" PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/vendor/glm 0.9.9.8")
target_include_directories("${CMAKE_PROJECT_NAME}" PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/vendor/entt-3.12.2/single_include/entt")
target_include_directories("${CMAKE_PROJECT_NAME}" PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/vendor/glad3.3core/include")
target_include_directories("${CMAKE_PROJECT_NAME}" PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/vendor/SDL2-2.26.1/include")

add_definitions(
		-DGLM_FORCE_INLINE
		-DPHYSECS_EXPORTS
)