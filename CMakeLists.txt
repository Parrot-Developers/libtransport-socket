cmake_minimum_required(VERSION 3.15)
project(transport-socket VERSION 1.0)

set(LIB_SOURCES
            src/tskt.c
            src/tskt_impl.c
            src/tskt_pomp.c
            src/tskt_resolv.c
            src/tskt_resolv_impl.c
)

add_library(${PROJECT_NAME} ${LIB_SOURCES})

# checks if set up rpath exists for install
if(COMMAND set_up_rpath)
    set_up_rpath()
else()
    message("Set up rpath not defined!")
endif()

option(BUILD_SHARED_LIBS "Build using shared libraries" ON)
set_target_properties(transport-socket PROPERTIES
                    POSITION_INDEPENDENT_CODE ${BUILD_SHARED_LIBS}
)

target_include_directories(${PROJECT_NAME} PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>
)


target_compile_definitions(${PROJECT_NAME} PRIVATE "TSKT_API_EXPORTS")
target_compile_definitions(${PROJECT_NAME} PRIVATE "_GNU_SOURCE")
target_compile_options(${PROJECT_NAME} PRIVATE "-fvisibility=hidden")
target_compile_options(${PROJECT_NAME} PRIVATE "-std=gnu99")

if(WIN32)
  target_compile_definitions(${PROJECT_NAME} PRIVATE "_WIN32_WINNT=0x0600")
  target_link_libraries(${PROJECT_NAME} PRIVATE ws2_32 futils pomp ulog transport-packet)
else()
  target_link_libraries(${PROJECT_NAME} PRIVATE futils pomp ulog transport-packet rt)
endif()

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}-targets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

