cmake_minimum_required(VERSION 3.15)
project(libtransport-socket VERSION 1.0)

set(LIB_SOURCES
            src/tskt.c
            src/tskt_impl.c
            src/tskt_pomp.c
            src/tskt_resolv.c
            src/tskt_resolv_impl.c
)

add_library(transport-socket ${LIB_SOURCES})

option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

target_include_directories(transport-socket PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>
)

target_compile_definitions(transport-socket PRIVATE "TSKT_API_EXPORTS")
target_compile_definitions(transport-socket PRIVATE "_GNU_SOURCE")
target_compile_options(transport-socket PRIVATE "-fvisibility=hidden")
target_compile_options(transport-socket PRIVATE "-std=gnu99")

if(WIN32)
  target_compile_definitions(transport-socket PRIVATE "_WIN32_WINNT=0x0600")
  target_link_libraries(transport-socket ws2_32)
endif()

target_link_libraries(transport-socket futils pomp ulog transport-packet)