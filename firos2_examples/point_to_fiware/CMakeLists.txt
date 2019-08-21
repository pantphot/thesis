project(point_to_ngsiv2)
cmake_minimum_required(VERSION 2.9)

include(CheckCXXCompilerFlag)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANG OR
        CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    check_cxx_compiler_flag(--std=c++11 SUPPORTS_CXX11)
    if(SUPPORTS_CXX11)
        add_compile_options(--std=c++11)
    else()
        message(FATAL_ERROR "Compiler doesn't support C++11")
    endif()
endif()

find_package(fastcdr)
find_package(fastrtps)

add_definitions(-DEPROSIMA_USER_DLL_EXPORT  -DBUILD_SHARED_LIBS)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(BUILD_SHARED_LIBS TRUE)

# cURLpp as thirdparty
set(CURLPP_DIR ../../thirdparty/curlpp)
file(GLOB_RECURSE CURLPP_HEADERS ${CURLPP_DIR}/include/*)
file(GLOB_RECURSE CURLPP_SOURCES ${CURLPP_DIR}/src/*)

if(WIN32)
	set(CURL_BUILD_DIR ../../thirdparty/curl/builds/libcurl-vc14-x64-release-dll-ipv6-sspi-winssl)

    LINK_DIRECTORIES(${CURL_BUILD_DIR}/lib)
	LINK_DIRECTORIES(${CURL_BUILD_DIR}/bin)

    set(liblist libcurl)
else()
	set(liblist -L/usr/lib/x86_64-linux-gnu curlpp curl)
endif()

file(GLOB USER_LIB_SOURCES_CXX "*.cxx")
file(GLOB USER_LIB_SOURCES_CPP "userlib.cpp")

add_library(userlib SHARED ${USER_LIB_SOURCES_CXX} ${USER_LIB_SOURCES_CPP}
    ../../src/NGSIv2/idl/JsonNGSIv2.cxx
    ../../src/NGSIv2/idl/JsonNGSIv2PubSubTypes.cxx)

if(WIN32)
    add_executable(cb_create_point cb_create_point.cpp ${CURLPP_HEADERS} ${CURLPP_SOURCES})
    add_executable(cb_get_point cb_get_point.cpp ${CURLPP_HEADERS} ${CURLPP_SOURCES})
    target_include_directories(cb_create_point PUBLIC ${CURLPP_DIR}/include ${CURL_BUILD_DIR}/include)
    target_include_directories(cb_get_point PUBLIC ${CURLPP_DIR}/include ${CURL_BUILD_DIR}/include)
    configure_file(${CURL_BUILD_DIR}/bin/libcurl.dll ${CMAKE_CURRENT_BINARY_DIR}/Release/libcurl.dll COPYONLY)
else()
	add_executable(cb_create_point cb_create_point.cpp ${CURLPP_HEADERS} ${CURLPP_SOURCES})
	add_executable(cb_get_point cb_get_point.cpp ${CURLPP_HEADERS} ${CURLPP_SOURCES})
    target_include_directories(cb_create_point PUBLIC ${CURLPP_DIR}/include)
    target_include_directories(cb_get_point PUBLIC ${CURLPP_DIR}/include)
endif()

target_link_libraries(userlib fastrtps fastcdr ${CMAKE_DL_LIBS})
target_link_libraries(cb_create_point ${liblist})
target_link_libraries(cb_get_point ${liblist})

set_target_properties(userlib PROPERTIES OUTPUT_NAME "userlib")
set_target_properties(cb_create_point PROPERTIES OUTPUT_NAME "cb_create_point")
set_target_properties(cb_get_point PROPERTIES OUTPUT_NAME "cb_get_point")

install(TARGETS cb_get_point
    RUNTIME DESTINATION examples/${PROJECT_NAME}/build$<$<BOOL:${WIN32}>:/${CMAKE_BUILD_TYPE}>
    )

install(TARGETS cb_create_point
    RUNTIME DESTINATION examples/${PROJECT_NAME}/build$<$<BOOL:${WIN32}>:/${CMAKE_BUILD_TYPE}>
    )

install(TARGETS userlib
    LIBRARY DESTINATION examples/${PROJECT_NAME}/build$<$<BOOL:${WIN32}>:/${CMAKE_BUILD_TYPE}>
    )

install(FILES
    config$<$<BOOL:${WIN32}>:_win>.xml
    DESTINATION examples/${PROJECT_NAME}
    )

if(WIN32)
    install(FILES
    ${CURL_BUILD_DIR}/bin/libcurl.dll
        DESTINATION examples/${PROJECT_NAME}/build$<$<BOOL:${WIN32}>:/${CMAKE_BUILD_TYPE}>
        )
endif()

#cmake_minimum_required(VERSION 2.8.12)
