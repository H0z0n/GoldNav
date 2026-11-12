# Юнит-тесты engine-agnostic ядра. Каждый файл — отдельный исполняемый файл со
# своим main() (см. micro_test.h) — так проще диагностировать, какой именно
# тест упал, и не нужен внешний test-runner framework.

set(GOLDNAV_TEST_SOURCES
    test_binary_heap.cpp
    test_astar.cpp
    test_path_smoother.cpp
    test_nav_grid.cpp
    test_hull_filtered_graph_view.cpp
    test_nav_generator.cpp
    test_path_planner.cpp
    test_ground_mover.cpp
    test_path_follower.cpp
    test_dynamic.cpp
)

foreach(test_src ${GOLDNAV_TEST_SOURCES})
    get_filename_component(test_name ${test_src} NAME_WE)
    add_executable(${test_name} ${test_src})
    target_link_libraries(${test_name} PRIVATE goldnav_core)
    target_include_directories(${test_name} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/src
    )
    add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()
