if(ENABLE_DATA_TOOLS)
  file(GLOB_RECURSE TEST_FILES "${CMAKE_CURRENT_LIST_DIR}/test_*.cc")

  foreach(FULLFILENAME IN ITEMS ${TEST_FILES})
    file(RELATIVE_PATH FILENAME "${CMAKE_CURRENT_LIST_DIR}" ${FULLFILENAME})
    string(REPLACE ".cc" "" TESTNAME ${FILENAME})
    add_executable(gurka_${TESTNAME} EXCLUDE_FROM_ALL ${FILENAME})
    set_target_properties(gurka_${TESTNAME} PROPERTIES
      COMPILE_DEFINITIONS VALHALLA_SOURCE_DIR="${VALHALLA_SOURCE_DIR}/")
    target_link_libraries(gurka_${TESTNAME} valhalla gtest gmock)
    target_include_directories(gurka_${TESTNAME} PRIVATE
      ${VALHALLA_SOURCE_DIR}/third_party/protozero/include
      ${VALHALLA_SOURCE_DIR}/third_party/libosmium/include)
  endforeach()
endif()