//****************************************************************************
// Copyright 2022 Richard Hulme
//
// SPDX-License-Identifier: BSD-3-Clause
//
// Urloader for the RP2040
//
// Tries to start the flashloader if possible.  If not, tries to start the
// application.
//
// If all else fails (application is invalid and no update image can be found)
// the flashloader will drop back to bootrom bootloader.

#include <stdint.h>
#include "hardware/regs/m0plus.h"
#include "hardware/clocks.h"
#include "hardware/watchdog.h"
#include "hardware/resets.h"
#include "hardware/dma.h"
#include "pico/bootrom.h"
#include "pico/binary_info.h"
#include "urloader.h"

// Start addresses defined in linker script
extern void* __FLASHLOADER_START;
extern void* __APPLICATION_START;

bi_decl(bi_program_version_string("1.00"));

#define bl2crc(x)      (*((uint32_t*)(((uint32_t)(x) + 0xfc))))

//****************************************************************************
// This is normally provided as part of pico_stdlib so we have to provide it
// here if not we're not using it.
void exit(int ret)
{
    (void)ret;
    while(true)
        tight_loop_contents();
}

//****************************************************************************
// Replace the standard 'atexit' with an empty version to avoid pulling in
// additional code that we don't need anyway.
int atexit(void *a, void (*f)(void*), void *d)
{
    (void)a;
    (void)f;
    (void)d;
    return 0;
}

//****************************************************************************
// Calculate the CRC32 (no reflection, no final XOR) of a block of data.
// This makes use of the DMA sniffer to calculate the CRC for us.  Speed is
// not really a huge issue as most of the time we just need to check the
// boot2 image is valid (252 bytes) but using DMA ought to be faster than
// looping over the data without a lookup table and is certainly a lot smaller
// than the lookup table.
uint32_t crc32(const void *data, size_t len, uint32_t crc)
{
    // Nothing else is running on the system, so it doesn't matter which
    // DMA channel we use
    static const uint8_t channel = 0;

    uint8_t dummy;

    dma_channel_config c = dma_channel_get_default_config(channel);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_sniff_enable(&c, true);

    // Turn on CRC32 (non-bit-reversed data)
    dma_sniffer_enable(channel, 0x00, true);
    dma_hw->sniff_data = crc;

    dma_channel_configure(
        channel,
        &c,
        &dummy,
        data,
        len,
        true    // Start immediately
    );

    dma_channel_wait_for_finish_blocking(channel);

    return(dma_hw->sniff_data);
}

//****************************************************************************
// Start the application at the given address if its boot2 image is valid.
// Will not return unless the image is invalid
int startApplication(uint32_t imgStart)
{
    imgStart += XIP_BASE;

    if(crc32((const void*)imgStart, 252, 0xffffffff) == bl2crc(imgStart))
    {
        // Hold DMA block in reset again
        reset_block(RESETS_RESET_DMA_BITS);

        // Code appears to be OK so we can map the code's
        // vector table and jump to its start
        asm volatile (
        "mov r0, %[start]\n"
        "ldr r1, =%[vtable]\n"
        "str r0, [r1]\n"
        "ldmia r0, {r0, r1}\n"
        "msr msp, r0\n"
        "bx r1\n"
        :
        : [start] "r" (imgStart + 0x100), [vtable] "X" (PPB_BASE + M0PLUS_VTOR_OFFSET)
        :
        );
    }

    // We will only return if the main application couldn't be started
    return 0;
}

//****************************************************************************
int main(void)
{
    // Take DMA block out of reset so we can use it to calculate CRCs
    unreset_block_wait(RESETS_RESET_DMA_BITS);

    // Try to start the flashloader
    startApplication((uint32_t)&__FLASHLOADER_START);

    // If that fails (and we return), try to start the application but let
    // the application know the flashloader is invalid so it can try to
    // re-flash the flashloader (if possible)
    watchdog_hw->scratch[0] = URLOADER_BAD_FLASHLOADER;
    startApplication((uint32_t)&__APPLICATION_START);

    // Otherwise go to the bootrom bootloader as a last resort

    // Disable resuscitation or the reset into the bootloader doesn't work
    clocks_hw->resus.ctrl = 0;

    reset_usb_boot(0, 0);
}
