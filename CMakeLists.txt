cmake_minimum_required(VERSION "3.19.2")

project(Physecs)

set(PHYSECS_LIB Physecs)
set(PHYSECS_DEMO PhysecsDemo)

file(GLOB_RECURSE LIB_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
file(GLOB_RECURSE DEMO_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/demo/*.cpp")

add_library("${PHYSECS_LIB}" SHARED)
add_executable("${PHYSECS_DEMO}")

set_property(TARGET "${PHYSECS_LIB}" PROPERTY CXX_STANDARD 17)
set_property(TARGET "${PHYSECS_DEMO}" PROPERTY CXX_STANDARD 17)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

set_property(TARGET "${PHYSECS_LIB}" PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded")
set_property(TARGET "${PHYSECS_DEMO}" PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded")

set(GLAD_SRC "${CMAKE_CURRENT_SOURCE_DIR}/vendor/glad3.3core/src/glad.c")

target_sources("${PHYSECS_LIB}" PRIVATE ${LIB_SOURCES})
target_sources("${PHYSECS_DEMO}" PRIVATE ${DEMO_SOURCES} ${GLAD_SRC})

target_include_directories("${PHYSECS_LIB}" PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_include_directories("${PHYSECS_LIB}" PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/Physecs")
target_include_directories("${PHYSECS_LIB}" PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/Physecs/Joints")
target_include_directories("${PHYSECS_LIB}" PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/vendor/glm 0.9.9.8")
target_include_directories("${PHYSECS_LIB}" PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/vendor/entt-3.12.2/single_include/entt")

target_include_directories("${PHYSECS_DEMO}" PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/demo")
target_include_directories("${PHYSECS_DEMO}" PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/demo/Renderer")
target_include_directories("${PHYSECS_DEMO}" PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_include_directories("${PHYSECS_DEMO}" PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/vendor/glm 0.9.9.8")
target_include_directories("${PHYSECS_DEMO}" PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/vendor/entt-3.12.2/single_include/entt")
target_include_directories("${PHYSECS_DEMO}" PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/vendor/glad3.3core/include")
target_include_directories("${PHYSECS_DEMO}" PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/vendor/SDL2-2.26.1/include")

target_link_directories("${PHYSECS_DEMO}" PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/vendor/SDL2-2.26.1/lib/x64")

target_link_libraries("${PHYSECS_DEMO}" "${PHYSECS_LIB}" SDL2 SDL2main opengl32)

add_definitions(
		-DGLM_FORCE_INLINE
		-DTINYGLTF_IMPLEMENTATION
		-DTINYGLTF_NO_STB_IMAGE_WRITE
		-DPHYSECS_EXPORTS
)

add_custom_command(TARGET "${PHYSECS_DEMO}" POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory_if_different
    ${CMAKE_CURRENT_SOURCE_DIR}/res
    ${CMAKE_CURRENT_BINARY_DIR}
)