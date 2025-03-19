include(CMakePrintHelpers)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


################################
# Boost subset
################################

# https://github.com/espressif/esp-idf/issues/9929

if (NOT CMAKE_BUILD_EARLY_EXPANSION)

# https://matgomes.com/integrate-google-test-into-cmake/
include(FetchContent)

# https://cmake.org/cmake/help/latest/module/FindGit.html
find_package(Git REQUIRED)
if(Git_FOUND)
    message("Git found: ${GIT_EXECUTABLE}")
endif()

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK "${CCACHE_PROGRAM}")
    message(STATUS "ccache found and enabled")
endif()

# Boost version
set(BOOST_VERSION 1.87.0)

# Boost Core (Boost license)
FetchContent_Declare(
    boost_core
    GIT_REPOSITORY https://github.com/boostorg/core.git
    GIT_TAG boost-${BOOST_VERSION}
)
FetchContent_MakeAvailable(boost_core)
set(BOOST_CORE_INCLUDE_PATH "${boost_core_SOURCE_DIR}/include")
cmake_print_variables(BOOST_CORE_INCLUDE_PATH)

# Boost Spirit (Boost license)
FetchContent_Declare(
    boost_spirit
    GIT_REPOSITORY https://github.com/boostorg/spirit.git
    GIT_TAG boost-${BOOST_VERSION}
)
FetchContent_MakeAvailable(boost_spirit)
set(BOOST_SPIRIT_INCLUDE_PATH "${boost_spirit_SOURCE_DIR}/include")
cmake_print_variables(BOOST_SPIRIT_INCLUDE_PATH)

# Boost Fusion (Boost license)
FetchContent_Declare(
    boost_fusion
    GIT_REPOSITORY https://github.com/boostorg/fusion.git
    GIT_TAG boost-${BOOST_VERSION}
)
FetchContent_MakeAvailable(boost_fusion)
set(BOOST_FUSION_INCLUDE_PATH "${boost_fusion_SOURCE_DIR}/include")
cmake_print_variables(BOOST_FUSION_INCLUDE_PATH)

# Boost Endian (Boost license)
FetchContent_Declare(
    boost_endian
    GIT_REPOSITORY https://github.com/boostorg/endian.git
    GIT_TAG boost-${BOOST_VERSION}
)
FetchContent_MakeAvailable(boost_endian)
set(BOOST_ENDIAN_INCLUDE_PATH "${boost_endian_SOURCE_DIR}/include")
cmake_print_variables(BOOST_ENDIAN_INCLUDE_PATH)

# Boost Config (Boost license)
FetchContent_Declare(
    boost_config
    GIT_REPOSITORY https://github.com/boostorg/config.git
    GIT_TAG boost-${BOOST_VERSION}
)
FetchContent_MakeAvailable(boost_config)
set(BOOST_CONFIG_INCLUDE_PATH "${boost_config_SOURCE_DIR}/include")
cmake_print_variables(BOOST_CONFIG_INCLUDE_PATH)

# Boost MPL (Boost license)
FetchContent_Declare(
    boost_mpl
    GIT_REPOSITORY https://github.com/boostorg/mpl.git
    GIT_TAG boost-${BOOST_VERSION}
)
FetchContent_MakeAvailable(boost_mpl)
set(BOOST_MPL_INCLUDE_PATH "${boost_mpl_SOURCE_DIR}/include")
cmake_print_variables(BOOST_MPL_INCLUDE_PATH)

# Boost Type Traits (Boost license)
FetchContent_Declare(
    boost_type_traits
    GIT_REPOSITORY https://github.com/boostorg/type_traits.git
    GIT_TAG boost-${BOOST_VERSION}
)
FetchContent_MakeAvailable(boost_type_traits)
set(BOOST_TYPE_TRAITS_INCLUDE_PATH "${boost_type_traits_SOURCE_DIR}/include")
cmake_print_variables(BOOST_TYPE_TRAITS_INCLUDE_PATH)

# Boost Utility (Boost license)
FetchContent_Declare(
    boost_utility
    GIT_REPOSITORY https://github.com/boostorg/utility.git
    GIT_TAG boost-${BOOST_VERSION}
)
FetchContent_MakeAvailable(boost_utility)
set(BOOST_UTILITY_INCLUDE_PATH "${boost_utility_SOURCE_DIR}/include")
cmake_print_variables(BOOST_UTILITY_INCLUDE_PATH)

# Boost Preprocessor (Boost license)
FetchContent_Declare(
    boost_preprocessor
    GIT_REPOSITORY https://github.com/boostorg/preprocessor.git
    GIT_TAG boost-${BOOST_VERSION}
)
FetchContent_MakeAvailable(boost_preprocessor)
set(BOOST_PREPROCESSOR_INCLUDE_PATH "${boost_preprocessor_SOURCE_DIR}/include")
cmake_print_variables(BOOST_PREPROCESSOR_INCLUDE_PATH)

# Boost Assert (Boost license)
FetchContent_Declare(
    boost_assert
    GIT_REPOSITORY https://github.com/boostorg/assert.git
    GIT_TAG boost-${BOOST_VERSION}
)
FetchContent_MakeAvailable(boost_assert)
set(BOOST_ASSERT_INCLUDE_PATH "${boost_assert_SOURCE_DIR}/include")
cmake_print_variables(BOOST_ASSERT_INCLUDE_PATH)

# Boost Static Assert (Boost license)
FetchContent_Declare(
    boost_static_assert
    GIT_REPOSITORY https://github.com/boostorg/static_assert.git
    GIT_TAG boost-${BOOST_VERSION}
)
FetchContent_MakeAvailable(boost_static_assert)
set(BOOST_STATIC_ASSERT_INCLUDE_PATH "${boost_static_assert_SOURCE_DIR}/include")
cmake_print_variables(BOOST_STATIC_ASSERT_INCLUDE_PATH)

# Boost Variant (Boost license)
FetchContent_Declare(
    boost_variant
    GIT_REPOSITORY https://github.com/boostorg/variant.git
    GIT_TAG boost-${BOOST_VERSION}
)
FetchContent_MakeAvailable(boost_variant)
set(BOOST_VARIANT_INCLUDE_PATH "${boost_variant_SOURCE_DIR}/include")
cmake_print_variables(BOOST_VARIANT_INCLUDE_PATH)

# Boost Type Index (Boost license)
FetchContent_Declare(
    boost_type_index
    GIT_REPOSITORY https://github.com/boostorg/type_index.git
    GIT_TAG boost-${BOOST_VERSION}
)
FetchContent_MakeAvailable(boost_type_index)
set(BOOST_TYPE_INDEX_INCLUDE_PATH "${boost_type_index_SOURCE_DIR}/include")
cmake_print_variables(BOOST_TYPE_INDEX_INCLUDE_PATH)

# Boost Container Hash (Boost license)
FetchContent_Declare(
    boost_container_hash
    GIT_REPOSITORY https://github.com/boostorg/container_hash.git
    GIT_TAG boost-${BOOST_VERSION}
)
FetchContent_MakeAvailable(boost_container_hash)
set(BOOST_CONTAINER_HASH_INCLUDE_PATH "${boost_container_hash_SOURCE_DIR}/include")
cmake_print_variables(BOOST_CONTAINER_HASH_INCLUDE_PATH)

# Boost Describe (Boost license)
FetchContent_Declare(
    boost_describe
    GIT_REPOSITORY https://github.com/boostorg/describe.git
    GIT_TAG boost-${BOOST_VERSION}
)
FetchContent_MakeAvailable(boost_describe)
set(BOOST_DESCRIBE_INCLUDE_PATH "${boost_describe_SOURCE_DIR}/include")
cmake_print_variables(BOOST_DESCRIBE_INCLUDE_PATH)

# Boost MP11 (Boost license)
FetchContent_Declare(
    boost_mp11
    GIT_REPOSITORY https://github.com/boostorg/mp11.git
    GIT_TAG boost-${BOOST_VERSION}
)
FetchContent_MakeAvailable(boost_mp11)
set(BOOST_MP11_INCLUDE_PATH "${boost_mp11_SOURCE_DIR}/include")
cmake_print_variables(BOOST_MP11_INCLUDE_PATH)

# Boost Detail (Boost license)
FetchContent_Declare(
    boost_detail
    GIT_REPOSITORY https://github.com/boostorg/detail.git
    GIT_TAG boost-${BOOST_VERSION}
)
FetchContent_MakeAvailable(boost_detail)
set(BOOST_DETAIL_INCLUDE_PATH "${boost_detail_SOURCE_DIR}/include")
cmake_print_variables(BOOST_DETAIL_INCLUDE_PATH)

# Boost Integer (Boost license)
FetchContent_Declare(
    boost_integer
    GIT_REPOSITORY https://github.com/boostorg/integer.git
    GIT_TAG boost-${BOOST_VERSION}
)
FetchContent_MakeAvailable(boost_integer)
set(BOOST_INTEGER_INCLUDE_PATH "${boost_integer_SOURCE_DIR}/include")
cmake_print_variables(BOOST_INTEGER_INCLUDE_PATH)

# Boost Throw Exception (Boost license)
FetchContent_Declare(
    boost_throw_exception
    GIT_REPOSITORY https://github.com/boostorg/throw_exception.git
    GIT_TAG boost-${BOOST_VERSION}
)
FetchContent_MakeAvailable(boost_throw_exception)
set(BOOST_THROW_EXCEPTION_INCLUDE_PATH "${boost_throw_exception_SOURCE_DIR}/include")
cmake_print_variables(BOOST_THROW_EXCEPTION_INCLUDE_PATH)

# Boost Range (Boost license)
FetchContent_Declare(
    boost_range
    GIT_REPOSITORY https://github.com/boostorg/range.git
    GIT_TAG boost-${BOOST_VERSION}
)
FetchContent_MakeAvailable(boost_range)
set(BOOST_RANGE_INCLUDE_PATH "${boost_range_SOURCE_DIR}/include")
cmake_print_variables(BOOST_RANGE_INCLUDE_PATH)

# Boost Optional (Boost license)
FetchContent_Declare(
    boost_optional
    GIT_REPOSITORY https://github.com/boostorg/optional.git
    GIT_TAG boost-${BOOST_VERSION}
)
FetchContent_MakeAvailable(boost_optional)
set(BOOST_OPTIONAL_INCLUDE_PATH "${boost_optional_SOURCE_DIR}/include")
cmake_print_variables(BOOST_OPTIONAL_INCLUDE_PATH)

# Boost Iterator (Boost license)
FetchContent_Declare(
    boost_iterator
    GIT_REPOSITORY https://github.com/boostorg/iterator.git
    GIT_TAG boost-${BOOST_VERSION}
)
FetchContent_MakeAvailable(boost_iterator)
set(BOOST_ITERATOR_INCLUDE_PATH "${boost_iterator_SOURCE_DIR}/include")
cmake_print_variables(BOOST_ITERATOR_INCLUDE_PATH)

# Boost Concept Check (Boost license)
FetchContent_Declare(
    boost_concept_check
    GIT_REPOSITORY https://github.com/boostorg/concept_check.git
    GIT_TAG boost-${BOOST_VERSION}
)
FetchContent_MakeAvailable(boost_concept_check)
set(BOOST_CONCEPT_CHECK_INCLUDE_PATH "${boost_concept_check_SOURCE_DIR}/include")
cmake_print_variables(BOOST_CONCEPT_CHECK_INCLUDE_PATH)

# Boost TypeOf (Boost license)
FetchContent_Declare(
    boost_typeof
    GIT_REPOSITORY https://github.com/boostorg/typeof.git
    GIT_TAG boost-${BOOST_VERSION}
)
FetchContent_MakeAvailable(boost_typeof)
set(BOOST_TYPEOF_INCLUDE_PATH "${boost_typeof_SOURCE_DIR}/include")
cmake_print_variables(BOOST_TYPEOF_INCLUDE_PATH)

endif()

################################
# Local code
################################

# Local header files here ONLY
file(GLOB_RECURSE TARGET_H
    *.h
    *.hpp
    *.inl
)

# Local source files here

set(TARGET_CAN_UTILS_SRC
        can/can_codec.cpp
        dbc/dbc_parser.cpp
        v2c/v2c_transcoder.cpp
)

set(TARGET_SRC
        example/example.cpp
        "${TARGET_CAN_UTILS_SRC}"  
)

if (NOT CMAKE_BUILD_EARLY_EXPANSION)

set(TARGET_INCLUDE_DIR 
    .
    "${BOOST_CORE_INCLUDE_PATH}"
    "${BOOST_SPIRIT_INCLUDE_PATH}"
    "${BOOST_FUSION_INCLUDE_PATH}"
    "${BOOST_ENDIAN_INCLUDE_PATH}"
    "${BOOST_CONFIG_INCLUDE_PATH}"
    "${BOOST_MPL_INCLUDE_PATH}"
    "${BOOST_TYPE_TRAITS_INCLUDE_PATH}"
    "${BOOST_UTILITY_INCLUDE_PATH}"
    "${BOOST_PREPROCESSOR_INCLUDE_PATH}"
    "${BOOST_ASSERT_INCLUDE_PATH}"
    "${BOOST_STATIC_ASSERT_INCLUDE_PATH}"
    "${BOOST_VARIANT_INCLUDE_PATH}"
    "${BOOST_TYPE_INDEX_INCLUDE_PATH}"
    "${BOOST_CONTAINER_HASH_INCLUDE_PATH}"
    "${BOOST_DESCRIBE_INCLUDE_PATH}"
    "${BOOST_MP11_INCLUDE_PATH}"
    "${BOOST_DETAIL_INCLUDE_PATH}"
    "${BOOST_INTEGER_INCLUDE_PATH}"
    "${BOOST_THROW_EXCEPTION_INCLUDE_PATH}"
    "${BOOST_RANGE_INCLUDE_PATH}"
    "${BOOST_OPTIONAL_INCLUDE_PATH}"
    "${BOOST_ITERATOR_INCLUDE_PATH}"
    "${BOOST_CONCEPT_CHECK_INCLUDE_PATH}"
    "${BOOST_TYPEOF_INCLUDE_PATH}"
)

else()

set(TARGET_INCLUDE_DIR .)

endif()


idf_component_register(SRCS "${TARGET_SRC}" "${TARGET_H}" INCLUDE_DIRS "${TARGET_INCLUDE_DIR}")
