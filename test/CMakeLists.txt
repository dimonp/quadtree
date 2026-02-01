find_package(GTest CONFIG REQUIRED)

add_executable(quadtree_test
    quadtree_test.cpp
    quadtree_adv_test.cpp
    quadtree_collector_test.cpp
    quadtree_comprehensive_test.cpp
    main.cpp
)
target_include_directories(quadtree_test PRIVATE quadtree)
target_include_directories(quadtree_test PRIVATE quadtree_math)
target_link_libraries(quadtree_test PRIVATE quadtree_project_options)
target_link_libraries(quadtree_test PRIVATE
    GTest::gtest_main
    GTest::gmock_main
    quadtree
    quadtree_math
)

include(GoogleTest)
gtest_discover_tests(quadtree_test)