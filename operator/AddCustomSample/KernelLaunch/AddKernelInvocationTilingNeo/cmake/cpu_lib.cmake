if(NOT DEFINED ENV{CMAKE_PREFIX_PATH})
    set(CMAKE_PREFIX_PATH ${ASCEND_CANN_PACKAGE_PATH}/tools/tikicpulib/lib/cmake)
endif()
find_package(tikicpulib REQUIRED)

add_library(ascendc_kernels_${RUN_MODE} SHARED
    ${KERNEL_FILES}
)

target_link_libraries(ascendc_kernels_${RUN_MODE} PRIVATE
    tikicpulib::${SOC_VERSION}
)

target_compile_definitions(ascendc_kernels_${RUN_MODE} PRIVATE
    $<$<BOOL:$<IN_LIST:${SOC_VERSION},${CUSTOM_ASCEND310P_LIST}>>:CUSTOM_ASCEND310P>
)

target_compile_options(ascendc_kernels_${RUN_MODE} PRIVATE
    -g
    -O0
    -std=c++17
)

target_include_directories(ascendc_kernels_${RUN_MODE} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

install(TARGETS ascendc_kernels_${RUN_MODE}
DESTINATION ${CMAKE_INSTALL_LIBDIR}
)