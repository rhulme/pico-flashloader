//****************************************************************************
// Copyright 2022 Richard Hulme
//
// SPDX-License-Identifier: BSD-3-Clause
//
// Urloader for the RP2040
//
#ifndef __URLOADER_INCL__
#define __URLOADER_INCL__

#include <stdint.h>

// If this value is in the watchdog's first scratch register
// (watchdog_hw->scratch[0]), when the main application starts, the urloader
// determined the flashloader is invalid.
static const uint32_t URLOADER_BAD_FLASHLOADER = 0x5b7c94de; // Randomly picked number

#endif // __URLOADER_INCL__
