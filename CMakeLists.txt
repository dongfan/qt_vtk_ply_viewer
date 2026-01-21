cmake_minimum_required(VERSION 3.26)
project(qt_vtk_ply_viewer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Qt ---
find_package(Qt6 REQUIRED COMPONENTS Widgets)
qt_standard_project_setup()

# --- VTK ---
# VTK 9.3 설치본은 보통 vtk-config.cmake로 제공되며,
# VTK_DIR 또는 CMAKE_PREFIX_PATH를 통해 찾습니다.
find_package(VTK REQUIRED COMPONENTS
  CommonCore
  CommonDataModel
  FiltersSources
  InteractionStyle
  RenderingCore
  RenderingOpenGL2
  GUISupportQt
  IOPLY
)

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/MainWindow.h
  src/MainWindow.cpp
)

target_link_libraries(${PROJECT_NAME} PRIVATE
  Qt6::Widgets
  VTK::CommonCore
  VTK::CommonDataModel
  VTK::FiltersSources
  VTK::InteractionStyle
  VTK::RenderingCore
  VTK::RenderingOpenGL2
  VTK::GUISupportQt
  VTK::IOPLY
)

# Qt on Windows: set /utf-8 for MSVC builds (optional but nice)
if(MSVC)
  target_compile_options(${PROJECT_NAME} PRIVATE /utf-8)
endif()

# VTK helper: ensure required compile definitions etc.
vtk_module_autoinit(
  TARGETS ${PROJECT_NAME}
  MODULES VTK::GUISupportQt VTK::RenderingOpenGL2
)

# --- Runtime DLL convenience ---
# VTK/Qt DLL을 PATH에 안 넣어도 실행되도록, 실행 파일 옆으로 DLL 복사(빌드 후)
# if(WIN32)
#   add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
#     COMMAND ${CMAKE_COMMAND} -E echo "Copying Qt/VTK runtime DLLs..."
#
#     # Qt DLLs (windeployqt)
#     COMMAND "C:/Qt/6.10.1/msvc2022_64/bin/windeployqt.exe"
#             "$<TARGET_FILE:${PROJECT_NAME}>"
#
#     # VTK DLLs: 설치 bin 폴더를 exe 폴더로 복사
#     COMMAND ${CMAKE_COMMAND} -E copy_directory
#             "C:/dev/vtk-qt/deps/vtk-install/bin"
#             "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
#
#     VERBATIM
#   )
# endif()
