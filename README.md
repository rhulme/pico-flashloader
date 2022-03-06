# pico-flashloader

This is the 'urloader' branch of the pico-flashloader project.  Please read the [`README`](https://github.com/rhulme/pico-flashloader/blob/master/README.md) of the main branch to understand the purpose of the project.  This branch explores the "two stage" process listed under 'Possible extensions' in the main README.

It may be desirable to update the flashloader in the field as well as the application.  This _could_ be done by the application simply erasing and then re-writing the flashloader but that's not power-fail safe.  If anything goes wrong between starting the erase sequence and writing the last bit of the flashloader, the device will not boot again until it can be plugged into a host computer and forcibly brought up in the internal USB bootloader.

To be able to safely update the flashloader, we need an additional boot stage before that that checks whether the flashloader image is OK.  If so, it boots the flashloader.  If not, it boots into the application.  The application would then have the chance to try updating the flashloader again.

This first stage is the 'urloader'.  Technically, of course, it's not actually the very first loader as that is in ROM and then there's the 2nd stage loader in the first 256 bytes of flash but it's the first bit that we can sensibly change.

The main changes in this branch are:
* Add the urloader code
* Adjust the flash ranges in [`memmap_defines.ld`](memmap_defines.ld)
* Change [`app.c`](app.c) to update the flashloader instead of triggering an update of the application.

The flashloader itself has not been changed with the exception of setting a scratch register to make the demonstration clearer.  This small section of code (search for `FLASHLOADER_TEST_VERSION` in [`flashloader.c`](flashloader.c)) can be removed from any production code.

## The urloader
The urloader tries to be simple as possible because it cannot be updated in the field.

The boot sequence is as follows (falling through to the next step if the current step fails):
1. If CRC of the flashloader is good, start the flashloader
1. If CRC of the application is good, start the application
1. Reboot into the USB bootloader

# Flash layout
The location and reserved size of the individual images are defined in [`memmap_defines.ld`](memmap_defines.ld).  In general, it should only be necessary to change the amount of flash reserved for the flashloader (see `__FLASHLOADER_LENGTH`).  That value should include space for growth due to future features.

The values used will be hard-coded into the urloader so they should be picked carefully.  It will not be possible to change them at a later date to allow more space for the flashloader as the urloader cannot be safely updated without using the USB update method or via SWD (and if these are easily available in the field, you probably don't need this project!).

# Demo application
Whereas [`app.c`](app.c) in the main branch was primarily for demonstration purposes, _some_ of the code in this branch should be re-used in your own applications, or at least studied more closely.

## Updating the flashloader
The code in `flashImage()` takes a binary image and overwrites the flashloader with its contents.
The CRC checked by the urloader is stored in the last four bytes of the first 256-byte page.  Therefore it is important to write this first page _last_.  If everything were written in sequence and something caused the flash process to fail after the first page, the urloader would consider the whole flashloader to be good and boot into an incomplete image.  By writing the first page last, we can be sure that everything else is good before 'activating' the image with the first page.

Depending on your circumstances, it may not be possible to store the entire flashloader image in a single buffer and/or erase and write the data in a single operation (everything else is stopped until it's all finished).  If this is the case, you will have to write your own update code but the sequence should follow the same pattern of writing the first page last.


## Building
This project has been built and tested using Linux and the Pico SDK v1.3.0 but it should work with any supported build environment
```
mkdir build
cd build
cmake ..
make -j8
```
(depending on your machine, you may wish to increase or decrease the `-j8` parameter to change the number of simultaneous threads to make best use of however many cores your build machine has)

Two versions of the flashloader are built, which just write a different value into one of the watchdog scratch registers.  This value is output by the main application to show which version of the flashloader is present (and thereby show the update worked).

## Testing
Once everything has been built, you'll have the following files in the build directory (there'll be plenty of others but these are the important ones):
```
pico_flashloader1.hex
pico_flashloader2.hex
FLASH_ME.uf2
```
Use the bootrom bootloader to install the `FLASH_ME.uf2` image on the device.  You should see the message
```
Scratch: 0x00000001
```
appear on the UART (make sure you have a terminal connected!).  This shows that "flashloader1" is installed.

Open the `pico_flashloader2.hex` file in a text editor, select everything and copy it to the clipboard.
Now paste it into the terminal.  It's quite large and depending on your terminal you may have to confirm you wish to paste so much.
The message `Received block` will be shown for every 1k of payload data that is received:
```
Received block
Received block
Updating flashloader and then rebooting
Scratch: 0x00000002
```
The 'Scratch' message shows that now "flashloader2" is installed.

You can repeat the procedure with the `pico_flashloader1.hex` file.

You can also test "killing" the flashloader.  Simply copy the following into your terminal:
```
:020000041000EA
:10100000FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF0
:00000001FF
```
You should see the following:
```
Updating flashloader and then rebooting
Rebooting in 1 second
Scratch: 0x5B7C94DE
Flashloader is invalid!
```
The urloader sets the scratch register to the magic number 0x5B7C94DE (`URLOADER_BAD_FLASHLOADER` defined in [`urloader.h`](urloader.h)) when it detects the flashloader is invalid.  The application can check for this value at startup and perform whatever operations are deemed necessary to successfully update the flashloader.
