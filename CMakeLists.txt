cmake_minimum_required(VERSION 3.5)
set(CMAKE_C_STANDARD 11)

set(PICO_PLATFORM rp2040)
set(PICO_BOARD pico)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

set(PROJECT flashloader_test)

project(${PROJECT} C CXX)

pico_sdk_init()

# Need at least SDK v1.3.0 (due to https://github.com/raspberrypi/pico-sdk/issues/573)
if(${PICO_SDK_VERSION_STRING} VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Pico SDK v1.3.0 or greater is required.  You have ${PICO_SDK_VERSION_STRING}")
endif()

################################################################################
# Helper function
function(set_linker_script TARGET script)
    target_link_directories(${TARGET} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    pico_set_linker_script(${TARGET} ${CMAKE_CURRENT_SOURCE_DIR}/${script})

    # Add dependencies on the 'included' linker scripts so that the target gets
    # rebuilt if they are changed
    pico_add_link_depend(${TARGET} ${CMAKE_CURRENT_SOURCE_DIR}/memmap_defines.ld)
    pico_add_link_depend(${TARGET} ${CMAKE_CURRENT_SOURCE_DIR}/memmap_default.ld)
endfunction()

################################################################################
# Urloader
set(URLOADER pico-urloader)

add_executable(${URLOADER})

target_sources(${URLOADER} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/urloader.c
        )

target_link_libraries(${URLOADER} PRIVATE
        hardware_clocks
        hardware_watchdog
        hardware_resets
        hardware_dma
        pico_platform
        pico_standard_link
        )

pico_add_uf2_output(${URLOADER})
target_compile_options(${URLOADER} PUBLIC -Wall -Wextra -Wno-ignored-qualifiers -Os)

# Use a separate linker script for the urloader to make sure it is built
# to run at the right location (right at the start) and cannot overflow into
# the flashloader's address space
set_linker_script(${URLOADER} memmap_urloader.ld)

set(URLOADER_UF2 ${CMAKE_CURRENT_BINARY_DIR}/${URLOADER}.uf2)
install(FILES ${URLOADER_UF2} DESTINATION ${CMAKE_INSTALL_PREFIX})

################################################################################
# Flashloader (test1)
set(FLASHLOADER1 pico-flashloader1)

add_executable(${FLASHLOADER1})

target_sources(${FLASHLOADER1} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/flashloader.c
        )

target_link_libraries(${FLASHLOADER1} PRIVATE
        hardware_structs
        hardware_sync
        hardware_flash
        hardware_watchdog
        hardware_resets
        hardware_xosc
        hardware_clocks
        hardware_pll
        hardware_dma
        pico_platform
        pico_standard_link
        pico_divider
        )

pico_add_uf2_output(${FLASHLOADER1})
pico_add_hex_output(${FLASHLOADER1})
pico_set_program_name(${FLASHLOADER1} ${FLASHLOADER1})
target_compile_options(${FLASHLOADER1} PRIVATE -Wall -Wextra -Wno-ignored-qualifiers -Os)
target_compile_definitions(${FLASHLOADER1} PRIVATE FLASHLOADER_TEST_VERSION=1)

# Use a separate linker script for the flashloader to make sure it is built
# to run at the right location and cannot overflow into the applications's
# address space
set_linker_script(${FLASHLOADER1} memmap_flashloader.ld)

set(FLASHLOADER1_UF2 ${CMAKE_CURRENT_BINARY_DIR}/${FLASHLOADER1}.uf2)

################################################################################
# Flashloader (test2)
set(FLASHLOADER2 pico-flashloader2)

add_executable(${FLASHLOADER2})

target_sources(${FLASHLOADER2} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/flashloader.c
        )

target_link_libraries(${FLASHLOADER2} PRIVATE
        hardware_structs
        hardware_sync
        hardware_flash
        hardware_watchdog
        hardware_resets
        hardware_xosc
        hardware_clocks
        hardware_pll
        hardware_dma
        pico_platform
        pico_standard_link
        pico_divider
        )

pico_add_uf2_output(${FLASHLOADER2})
pico_add_hex_output(${FLASHLOADER2})
pico_set_program_name(${FLASHLOADER2} ${FLASHLOADER2})
target_compile_options(${FLASHLOADER2} PRIVATE -Wall -Wextra -Wno-ignored-qualifiers -Os)
target_compile_definitions(${FLASHLOADER2} PRIVATE FLASHLOADER_TEST_VERSION=2)

# Use a separate linker script for the flashloader to make sure it is built
# to run at the right location and cannot overflow into the applications's
# address space
set_linker_script(${FLASHLOADER2} memmap_flashloader.ld)

set(FLASHLOADER2_UF2 ${CMAKE_CURRENT_BINARY_DIR}/${FLASHLOADER2}.uf2)

################################################################################
# Application
set(APP app)

add_executable(${APP}
        app.c
        )

target_compile_options(${APP} PRIVATE -Os)
target_compile_definitions(${APP} PRIVATE LED_DELAY_MS=500)
target_link_libraries(${APP} pico_stdlib hardware_watchdog hardware_flash)

pico_add_uf2_output(${APP})
pico_add_dis_output(${APP})

# Use a separate linker script for the application to make sure it is built
# to run at the right location (after the flashloader).
set_linker_script(${APP} memmap_application.ld)

set(APP_UF2 ${CMAKE_CURRENT_BINARY_DIR}/${APP}.uf2)

################################################################################
# Combine the urloader, flashloader and application into one flashable UF2 image
set(COMPLETE_UF2 ${CMAKE_CURRENT_BINARY_DIR}/FLASH_ME.uf2)

find_package (Python3 REQUIRED COMPONENTS Interpreter)
add_custom_command(OUTPUT ${COMPLETE_UF2} DEPENDS ${URLOADER} ${FLASHLOADER1} ${APP}
        COMMENT "Building full UF2 image"
        COMMAND ${Python3_EXECUTABLE}
                ${CMAKE_CURRENT_SOURCE_DIR}/uf2tool.py
                -o ${COMPLETE_UF2} ${URLOADER_UF2} ${FLASHLOADER1_UF2} ${APP_UF2}
        )

add_custom_target(${PROJECT} ALL DEPENDS ${COMPLETE_UF2})

install(FILES ${COMPLETE_UF2} DESTINATION ${CMAKE_INSTALL_PREFIX} )
