/*
BSD 3-Clause License

Copyright (c) 2021, NewTec GmbH
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


/**
 * @file Zumo32U4Specification.h
 * @author Luis Moser
 * @brief Zumo32U4Specification header
 * @date 08/23/2021
 * @addtogroup Zumo
 * @{
 */

#ifndef __ZUMO32U4Specification_H__
#define __ZUMO32U4Specification_H__

#include <Arduino.h>

 /** Simple struct for specifying Zumo commands */
template <int size>
struct ZumoCommand
{
    /** The binary buffer for the command data */
    uint8_t commandData[size];

    /** Size of command data buffer in bytes */
    uint8_t commandSize;
};

/** Simple struct for specifying Zumo data */
template <int size>
struct ZumoData
{
    /** The binary buffer for the response Zumo data */
    uint8_t data[size];

    /** Size of the response data in bytes */
    uint8_t dataSize;
};

/** Namespace for specifying all used Zumo32U4 commands, return codes and expected return values */
namespace Zumo32U4Specification
{
    /** Command for reading the software ID */
    static const uint8_t READ_SW_ID[] = {0x53};

    /** Command for reading the software version */
    static const uint8_t READ_SW_VERSION[] = {0x56};

    /** Command for reading the hardware version */
    static const uint8_t READ_HW_VERSION[] = {0x76};

    /** Command for getting the programmer type */
    static const uint8_t READ_PROGRAMMER_TYPE[] = {0x70};

    /** Command for getting the supported bootlaoder device codes */
    static const uint8_t READ_SUPPORTED_DEVICE_CODE[] = {0x74};

    /** Command for reading the signature */
    static const uint8_t READ_SIGNATURE[] = {0x73};

    /** Command for reading the low byte AVR fuse */
    static const uint8_t READ_LSB_FUSE[] = {0x46};

    /** Command for reading the high byte AVR fuse */
    static const uint8_t READ_MSB_FUSE[] = {0x4E};

    /** Command for reading the extended byte AVR fuse */
    static const uint8_t READ_EXTENDED_FUSE[] = {0x51};

    /** Command for checking if the bootloader supports block/page flashing */
    static const uint8_t CHECK_BLOCK_FLASH_SUPPORT[] = {0x62};

    /** Command for setting the current R/W flash memory/page address */
    static const uint8_t SET_MEMORY_ADDR[] = {0x41};

    /** Command for checking if the bootloader supports automatically incrementing byte addresses when flashing a page */
    static const uint8_t CHECK_AUTO_MEM_ADDR_INC_SUPPORT[] = {0x61};

    /** Command for setting the device type for flashing */
    static const uint8_t SET_DEVICE_TYPE[] = {0x54};

    /** Command for switching the bootloader into the programmer mode */
    static const uint8_t ENTER_PROGRAMMING_MODE[] = {0x50};

    /** Command for leaving the bootloader from the programmer mode */
    static const uint8_t EXIT_PROGRAMMING_MODE[] = {0x4C};

    /** Command for exiting the bootloader */
    static const uint8_t EXIT_BOOTLOADER_MODE[] = {0x45};

    /** Command for writing a memory page with 128 bytes into the flash/program memory */
    static const uint8_t WRITE_MEMORY_PAGE[] = {0x42, 0x00, 0x80, 0x46};

    /** Command for reading a memory page with 128 bytes from the flash/program memory */
    static const uint8_t READ_MEMORY_PAGE[] = {0x67, 0x00, 0x80, 0x46};

    /** Carriage return for successful return code */
    static const uint8_t RET_OK [] = {0x0D};

    /** The expected bootloader ID string */
    static const uint8_t EXPECTED_SOFTWARE_ID[] = {'C', 'A', 'T', 'E', 'R', 'I', 'N', '\0'};

    /** The expected bootloader version */
    
    static const uint8_t EXPECTED_SW_VERSION[] = {0x31, 0x30};

    /** The expected hardware version */
    static const uint8_t EXPECTED_HW_VERSION [] = {0x3F};

    /** The expected programmer type */
    static const uint8_t EXPECTED_PROGRAMMER_TYPE[] = {0x53};

    /** The expected supported device code */
    static const uint8_t EXPECTED_DEVICE_CODE[] = {0x44};

    /** The expected result when checking if bootloader supports auto incrementing page byte addresses */
    static const uint8_t  EXPECTED_SUPPORTS_AUTO_MEM_ADDR_INC[] = {0x59};

    /** The expected block size result in bytes when checking if bootloader supports page/block flashing */
    static const uint8_t EXPECTED_BLOCK_BUFFER_SIZE[] = {0x59};

    /** The expected AVR low byte fuse value */
    static const uint8_t EXPECTED_LSB_FUSE_VALUE [] = {0xFF};

    /** The expected AVR high byte fuse value */
    static const uint8_t EXPECTED_MSB_FUSE_VALUE[] = {0xD0};

    /** The expected AVR extended byte fuse value */
    static const uint8_t EXPECTED_EXTENDED_FUSE_VALUE[] = {0xC8};

    /** The expected signature value */
    static const uint8_t EXPECTED_SIGNATURE[] = {0x87, 0x95, 0x1E};
};

#endif /** __ZUMO32U4Specification_H__ */
/** @} */
