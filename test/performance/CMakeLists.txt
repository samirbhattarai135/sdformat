set(TEST_TYPE "PERFORMANCE")

set(tests
  frame_semantics.cc
  parser_urdf.cc
)

# frame_semantics.cc exercises resolvePose, which is internal and not
# exported from the shared library, so these tests need the private headers
# and the object library used by the unit tests.
gz_build_tests(TYPE ${TEST_TYPE}
               SOURCES ${tests}
               LIB_DEPS library_for_tests
               INCLUDE_DIRS
                 ${PROJECT_SOURCE_DIR}/test
                 ${PROJECT_SOURCE_DIR}/src
               ENVIRONMENT SDF_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX})
