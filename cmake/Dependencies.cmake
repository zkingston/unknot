find_package(Eigen3 REQUIRED NO_MODULE)

# fmt
FetchContent_Declare(fmt
  GIT_REPOSITORY https://github.com/fmtlib/fmt.git
  GIT_TAG 11.1.4
)
FetchContent_MakeAvailable(fmt)

# JSON
FetchContent_Declare(json
  GIT_REPOSITORY https://github.com/nlohmann/json
  GIT_TAG v3.11.3
)
FetchContent_MakeAvailable(json)

# cxxopts
FetchContent_Declare(cxxopts
  GIT_REPOSITORY https://github.com/jarro2783/cxxopts.git
  GIT_TAG v3.2.1
)
FetchContent_MakeAvailable(cxxopts)

if(UNKNOT_VIZ)

  # Setup glad
  FetchContent_Declare(
    glad
    GIT_REPOSITORY "https://github.com/Dav1dde/glad"
    GIT_TAG "v0.1.36"
    GIT_PROGRESS TRUE
    GIT_SHALLOW TRUE
  )
  FetchContent_MakeAvailable(glad)

  # Setup GLFW
  set(GLFW_BUILD_EXAMPLES OFF CACHE INTERNAL "")
  set(GLFW_BUILD_TESTS OFF CACHE INTERNAL "")
  set(GLFW_BUILD_DOCS OFF CACHE INTERNAL "")
  FetchContent_Declare(
    glfw
    GIT_REPOSITORY "https://github.com/glfw/glfw"
    GIT_TAG "3.3.8"
    GIT_PROGRESS TRUE
    GIT_SHALLOW TRUE

  )
  FetchContent_MakeAvailable(glfw)

  # Setup ImGui
  FetchContent_Declare(
    imgui
    GIT_REPOSITORY "https://github.com/ocornut/imgui"
    GIT_TAG "v1.91.5"
    GIT_PROGRESS TRUE
    GIT_SHALLOW TRUE
  )
  FetchContent_MakeAvailable(imgui)
  set(IMGUI_SOURCE
    ${CMAKE_BINARY_DIR}/_deps/imgui-src/imgui.cpp
    # ${CMAKE_BINARY_DIR}/_deps/imgui-src/imgui_demo.cpp
    ${CMAKE_BINARY_DIR}/_deps/imgui-src/imgui_draw.cpp
    ${CMAKE_BINARY_DIR}/_deps/imgui-src/imgui_tables.cpp
    ${CMAKE_BINARY_DIR}/_deps/imgui-src/imgui_widgets.cpp
	  ${CMAKE_BINARY_DIR}/_deps/imgui-src/backends/imgui_impl_glfw.cpp
	  ${CMAKE_BINARY_DIR}/_deps/imgui-src/backends/imgui_impl_opengl3.cpp
  )
  add_library(imgui STATIC ${IMGUI_SOURCE})
  target_include_directories(imgui PUBLIC "${CMAKE_BINARY_DIR}/_deps/imgui-src/;${CMAKE_BINARY_DIR}/_deps/imgui-src/backends/")
  target_link_libraries(imgui PUBLIC glfw glad)

  # Setup ImPlot
  FetchContent_Declare(
    implot
    GIT_REPOSITORY "https://github.com/epezent/implot"
    GIT_TAG "v0.16"
    GIT_PROGRESS TRUE
    GIT_SHALLOW TRUE
  )
  FetchContent_MakeAvailable(implot)
  set(IMPLOT_SOURCE
    ${CMAKE_BINARY_DIR}/_deps/implot-src/implot.cpp
    # ${CMAKE_BINARY_DIR}/_deps/implot-src/implot_demo.cpp
    ${CMAKE_BINARY_DIR}/_deps/implot-src/implot_items.cpp
  )
  add_library(implot STATIC ${IMPLOT_SOURCE})
  target_link_libraries(implot PUBLIC imgui)
  target_include_directories(imgui PUBLIC "${CMAKE_BINARY_DIR}/_deps/implot-src/")

  # Setup ImPlot3d
  FetchContent_Declare(
    implot3d
    GIT_REPOSITORY "https://github.com/brenocq/implot3d"
    GIT_TAG v0.2
    GIT_PROGRESS TRUE
    GIT_SHALLOW TRUE
  )
  FetchContent_MakeAvailable(implot3d)
  set(IMPLOT_SOURCE
    ${CMAKE_BINARY_DIR}/_deps/implot3d-src/implot3d.cpp
    # ${CMAKE_BINARY_DIR}/_deps/implot3d-src/implot3d_demo.cpp
    ${CMAKE_BINARY_DIR}/_deps/implot3d-src/implot3d_items.cpp
    ${CMAKE_BINARY_DIR}/_deps/implot3d-src/implot3d_meshes.cpp
  )
  add_library(implot3d STATIC ${IMPLOT_SOURCE})
  target_include_directories(implot3d PUBLIC "${CMAKE_BINARY_DIR}/_deps/implot3d-src/")
  target_link_libraries(implot3d PUBLIC imgui)
endif()
