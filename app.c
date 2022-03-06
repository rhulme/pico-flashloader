//****************************************************************************
// Copyright 2021 Richard Hulme
//
// SPDX-License-Identifier: BSD-3-Clause
//
// Demo application to test the flashloader.
// Listens on the default UART for an Intel hex file containing a new
// application.  This is stored in flash and the system is rebooted into the
// flashloader which overwrites the existing application with the new image
// and boots into it.
// Because the flashloader is not overwriting itself, it is power-fail safe.
//
// This code is for demonstration purposes.  There is not very much
// error-checking and attempts have been made to keep the final code size
// small (e.g. not using printf)

#include <string.h>
#include "pico/stdlib.h"
#include "hardware/sync.h"
#include "hardware/flash.h"
#include "hardware/watchdog.h"
#include "hardware/structs/watchdog.h"
#include "flashloader.h"
#include "urloader.h"

extern const void const * __FLASHLOADER_START;
extern const void const * __FLASHLOADER_LENGTH;

// __FLASHLOADER_START and __FLASHLOADER_LENGTH are a symbols defined in a
// linker script so they appear as addresses rather than integer values.
// We'll define a couple of helpers to get them into the form we want them
// and make the code that uses them more readable.
#define FLASHLOADER_START  ((uint32_t)&__FLASHLOADER_START)
#define FLASHLOADER_LENGTH ((uint32_t)&__FLASHLOADER_LENGTH)

#ifndef PICO_DEFAULT_LED_PIN
    #error This example needs a board with an LED
#endif

#ifndef LED_DELAY_MS
    #error LED_DELAY_MS must be defined!
#endif

// Intel HEX record
typedef struct
{
    uint8_t  count;
    uint16_t addr;
    uint8_t  type;
    uint8_t  data[256];
}tRecord;

// Intel HEX record types
static const uint8_t TYPE_DATA      = 0x00;
static const uint8_t TYPE_EOF       = 0x01;
static const uint8_t TYPE_EXTSEG    = 0x02;
static const uint8_t TYPE_STARTSEG  = 0x03;
static const uint8_t TYPE_EXTLIN    = 0x04;
static const uint8_t TYPE_STARTLIN  = 0x05;

// Buffer to hold the incoming data before flashing
// (unfortunately it doesn't seem to be possible to use FLASHLOADER_LENGTH)
uint8_t flashbuf[16384];

//****************************************************************************
bool repeating_timer_callback(struct repeating_timer *t)
{
    (void)t;
    gpio_xor_mask(1 << PICO_DEFAULT_LED_PIN);
    return true;
}

//****************************************************************************
// Simple CRC32 (no reflection, no final XOR) implementation.
// This can be done with a lookup table or using the DMA sniffer too.
uint32_t crc32(const uint8_t *data, uint32_t len, uint32_t crc)
{
    while(len--)
    {
        crc ^= (*data++ << 24);

        for(int bit = 0; bit < 8; bit++)
        {
            if(crc & (1L << 31))
                crc = (crc << 1) ^ 0x04C11DB7;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}

//****************************************************************************
// Converts an ASCII hex character into its binary representation.
// The existing value is shifted across one nibble before the new value is
// stored in the lower nibble.
// Returns non-zero if the character could be converted
int hex2nibble(char c, uint8_t* value)
{
    int success = 0;

    if(c >= '0' && c <= '9')
    {
        *value <<= 4;
        *value |= (uint8_t)(c - '0');
        success = 1;
    }
    else
    {
        c |= 32;
        if(c >= 'a' && c <= 'z')
        {
            *value <<= 4;
            *value |= (uint8_t)(c - 'a') + 10;
            success = 1;
        }
    }

    return success;
}

//****************************************************************************
// Converts two ASCII hex characters to an 8-bit binary value.
// Returns non-zero if valid hex characters were found
int parseHex(const char* str, uint8_t* value)
{
    int success;

    *value = 0;
    success = hex2nibble(*str++, value) && hex2nibble(*str, value);

    return success;
}

//****************************************************************************
// Converts an Intel hex record in text form to a binary representation.
// Returns non-zero if the text could be parsed successfully
int processRecord(const char* line, tRecord* record)
{
    int     success = 0;
    int     offset = 0;
    uint8_t value;
    uint8_t data[256 + 5]; // Max payload 256 bytes plus 5 for fields
    uint8_t checksum = 0;

    while(*line && (*line != ':'))
        line++;

    if(*line++ == ':')
    {
        while(parseHex(line, &value) && (offset < sizeof(data)))
        {
            data[offset++] = value;
            checksum += value;
            line += 2;
        }
    }

    // Checksum is two's-complement of the sum of the previous bytes so
    // final checksum should be zero if everything was OK.
    if((offset > 0) && (checksum == 0))
    {
        record->count = data[0];
        record->addr  = data[2] | (data[1] << 8);
        record->type  = data[3];
        memcpy(record->data, &data[4], data[0]);
        success = 1;
    }

    return success;
}

//****************************************************************************
// Overwrite the flashloader with the given image then reboot if successful
void flashImage(const uint8_t* data, uint32_t length)
{
    // Round erase length up to next 4096 byte boundary
    uint32_t eraseLength = (length + 4095) & 0xfffff000;
    uint32_t status;
    uint32_t offset = 256;
    uint32_t success = 0;

    // Make sure the image provided will fit in the available space
    if(eraseLength > FLASHLOADER_LENGTH)
        uart_puts(PICO_DEFAULT_UART_INSTANCE, "Flashloader image is too big!\r\n");
    else
    {
        uart_puts(PICO_DEFAULT_UART_INSTANCE, "Updating flashloader and then rebooting\r\n");

        status = save_and_disable_interrupts();

        flash_range_erase(FLASHLOADER_START, eraseLength);

        // Write everything except the first page.  If there's any kind
        // of power failure during writing, this will prevent anything
        // trying to boot the partially flashed image

        // Get total number of pages - 1 (because we're flashing the first page
        // separately)
        for(uint32_t pages = ((length - 1) / 256); pages > 0; pages--)
        {
            flash_range_program(FLASHLOADER_START + offset,
                                data + offset,
                                256);
            offset += 256;
        }

        // Verify everything flashed so far
        if(memcmp((void*)(XIP_BASE + FLASHLOADER_START + 256), data + 256, offset - 256) == 0)
        {
            // Now flash the first page which is the boot2 image with CRC.
            flash_range_program(FLASHLOADER_START,
                                data,
                                256);

            // If the first page was also good, everything is good.
            // (if not, there'll be a CRC mismatch so the image won't be
            // started anyway)
            success = (memcmp((void*)(XIP_BASE + FLASHLOADER_START), data, 256) == 0);
        }

        restore_interrupts(status);

        if(success)
        {
            uart_puts(PICO_DEFAULT_UART_INSTANCE, "Rebooting in 1 second\r\n");

            watchdog_reboot(0x00000000, 0x00000000, 1000);

            // Wait for the reset
            while(true)
                tight_loop_contents();
        }
        else
            uart_puts(PICO_DEFAULT_UART_INSTANCE, "Flash verification failed!\r\n");
    }
}

//****************************************************************************
// Reads a line of text from the standard UART into the given buffer and
// returns when a line-feed or carriage-return is detected.
char* getLine(char* buffer)
{
    char c;
    char* ptr = buffer;

    do
    {
        c = uart_getc(PICO_DEFAULT_UART_INSTANCE);

        if((c != '\n') && (c != '\r'))
            *ptr++ = c;
        else
            *ptr++ = 0;
    }while((c != '\n') && (c != '\r'));

    return buffer;
}


//****************************************************************************
// Reads an Intel hex file from the standard UART, stores it in flash then
// triggers the flashloader to overwrite the existing application with the
// new image.
void readIntelHex()
{
    uint32_t      offset = 0;
    char          line[1024];
    uint32_t      count = 0;

    while (true)
    {
        tRecord rec;

        if(processRecord(getLine(line), &rec))
        {
            switch(rec.type)
            {
                case TYPE_DATA:
                    memcpy(&flashbuf[offset], rec.data, rec.count);
                    offset += rec.count;
                    offset %= 65536;
                    if((offset % 1024) == 0)
                        uart_puts(PICO_DEFAULT_UART_INSTANCE, "Received block\r\n");
                    break;

                case TYPE_EOF:
                    flashImage(flashbuf, offset);
                    break;

                case TYPE_EXTSEG:
                case TYPE_STARTSEG:
                case TYPE_STARTLIN:
                    // Ignore these types.  They aren't important for this demo
                    break;

                case TYPE_EXTLIN:
                    // Move to the start of the data buffer
                    offset = 0;
                    break;

                default:
                    break;
            }
            count++;
        }
    }
}

//****************************************************************************
void printHex(uint32_t value)
{
    static const char arr[] = "0123456789ABCDEF";

    // 32 bits == 8 hex chars + 1 for terminating NULL
    char buf[8 + 1];
    buf[sizeof(buf)-1] = '\0';

    for(int i = 0; i < sizeof(buf)-1; i++)
    {
        buf[i] = arr[value >> 28];
        value <<= 4;
    }

    uart_puts(PICO_DEFAULT_UART_INSTANCE, buf);
}

//****************************************************************************
// Entry point - start flashing the on-board LED and wait for a new
// flashloader image.
int main()
{
    gpio_set_function(PICO_DEFAULT_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(PICO_DEFAULT_UART_RX_PIN, GPIO_FUNC_UART);

    uart_init(PICO_DEFAULT_UART_INSTANCE, 115200);

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    struct repeating_timer timer;
    add_repeating_timer_ms(LED_DELAY_MS, repeating_timer_callback, NULL, &timer);

    uart_puts(PICO_DEFAULT_UART_INSTANCE, "Scratch: 0x");
    printHex(watchdog_hw->scratch[0]);
    uart_puts(PICO_DEFAULT_UART_INSTANCE, "\r\n");

    if(watchdog_hw->scratch[0] == FLASH_APP_UPDATED)
    {
        uart_puts(PICO_DEFAULT_UART_INSTANCE, "Application just updated!\r\n");
        watchdog_hw->scratch[0] = 0;
    }

    if(watchdog_hw->scratch[0] == URLOADER_BAD_FLASHLOADER)
        uart_puts(PICO_DEFAULT_UART_INSTANCE, "Flashloader is invalid!\r\n");

    readIntelHex();

    return 0;
}
