cmake_minimum_required(VERSION 3.19)
project(sdn_analyse_cpp LANGUAGES CXX)

find_package(Qt6 6.5 REQUIRED COMPONENTS Core)

qt_standard_project_setup()

qt_add_executable(sdn_analyse_cpp
    main.cpp
)

target_link_libraries(sdn_analyse_cpp
    PRIVATE
        Qt::Core
)

install(TARGETS sdn_analyse_cpp
    BUNDLE  DESTINATION .
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
