/* MIT License
 *
 * Copyright (c) 2023 - 2024 Andreas Merkle <web@blue-andi.de>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/*******************************************************************************
    DESCRIPTION
*******************************************************************************/
/**
 * @brief  Specification of the Commands of the CATERINA Bootloader.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 *
 * @addtogroup Service
 *
 * @{
 */
#ifndef CATERINA_SPECIFICATION_H
#define CATERINA_SPECIFICATION_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/

#include <stdint.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** Enumeration of the command IDs. */
enum COMMAND_ID : uint8_t
{
    CMD_ID_READ_SW_ID = 0U,        /**< Read software ID. */
    CMD_ID_READ_SW_VERSION,        /**< Read software version. */
    CMD_ID_READ_PROGRAMMER_TYPE,   /**< Read programmer type. */
    CMD_ID_CHECK_AUTOINCREMENT,    /**< Check Autoincrement support. */
    CMD_ID_CHECK_BLOCK_SUPPORT,    /**< Check block support. */
    CMD_IF_READ_SUPPORTED_DEVICES, /**< Read supported devices. */
    CMD_ID_SELECT_DEVICE,          /**< Select device. */
    CMD_ID_ENTER_PROG_MODE,        /**< Enter programming mode. */
    CMD_ID_READ_SIGNATURE,         /**< Read signature. */
    CMD_ID_READ_FUSE,              /**< Read fuse. */
    CMD_ID_READ_HIGH_FUSE,         /**< Read high fuse. */
    CMD_ID_READ_EXT_FUSE,          /**< Read extended fuse. */
    CMD_ID_SET_ADDRESS,            /**< Set address. */
    CMD_ID_WRITE_FLASH,            /**< Write flash. */
    CMD_ID_READ_FLASH,             /**< Read flash. */
    CMD_ID_LEAVE_PROG_MODE,        /**< Leave programming mode. */
    CMD_ID_LEAVE_BOOTLOADER,       /**< Leave bootloader. */
    CMD_ID_END                     /**< End of command IDs. Must always be the last command in the enum. */
};

/** Length of a memory address. */
static const uint8_t MEM_ADDR_LENGTH = 2U;

/** Max. length of a memory block. */
static const uint8_t MAX_MEM_BLOCK_LENGTH = 128U;

/** Command lengths in bytes. */
static constexpr uint8_t CMD_LENGTHS[CMD_ID_END] = {1U, 1U, 1U, 1U, 1U, 1U, 2U, 1U, 1U, 1U, 1U, 1U, 1U, 4U, 4U, 1U, 1U};

/** Response lengths in bytes. */
static constexpr uint8_t RSP_LENGTHS[CMD_ID_END] = {7U, 2U, 1U, 1U, 3U, 1U, 1U, 1U, 3U, 1U, 1U, 1U, 1U, 1U, 0U, 1U, 1U};

/** Response: OK. */
static const uint8_t RSP_OK = {'\r'};

/** Command: Read software ID. */
static const uint8_t CMD_READ_SW_ID[CMD_LENGTHS[CMD_ID_READ_SW_ID]] = {'S'};

/** Response: Read software ID. */
static const uint8_t RSP_READ_SW_ID[RSP_LENGTHS[CMD_ID_READ_SW_ID]] = {'C', 'A', 'T', 'E', 'R', 'I', 'N'};

/** Command: Read software version. */
static const uint8_t CMD_READ_SW_VERSION[CMD_LENGTHS[CMD_ID_READ_SW_VERSION]] = {'V'};

/** Response: Read software version. */
static const uint8_t RSP_READ_SW_VERSION[RSP_LENGTHS[CMD_ID_READ_SW_VERSION]] = {'1', '0'};

/** Command: Read programmer type. */
static const uint8_t CMD_READ_PROGRAMMER_TYPE[CMD_LENGTHS[CMD_ID_READ_PROGRAMMER_TYPE]] = {'p'};

/** Response: Read programmer type. */
static const uint8_t RSP_READ_PROGRAMMER_TYPE[RSP_LENGTHS[CMD_ID_READ_PROGRAMMER_TYPE]] = {'S'};

/** Command: Check Autoincrement support. */
static const uint8_t CMD_CHECK_AUTOINCREMENT[CMD_LENGTHS[CMD_ID_CHECK_AUTOINCREMENT]] = {'a'};

/** Response: Check Autoincrement support. */
static const uint8_t RSP_CHECK_AUTOINCREMENT[RSP_LENGTHS[CMD_ID_CHECK_AUTOINCREMENT]] = {'Y'};

/** Command: Check block support. */
static const uint8_t CMD_CHECK_BLOCK_SUPPORT[CMD_LENGTHS[CMD_ID_CHECK_BLOCK_SUPPORT]] = {'b'};

/** Response: Check block support. */
static const uint8_t RSP_CHECK_BLOCK_SUPPORT[RSP_LENGTHS[CMD_ID_CHECK_BLOCK_SUPPORT]] = {'Y', 0x00, 0x80};

/** Command: Read supported devices. */
static const uint8_t CMD_READ_SUPPORTED_DEVICES[CMD_LENGTHS[CMD_IF_READ_SUPPORTED_DEVICES]] = {'t'};

/** Response: Read supported devices. */
static const uint8_t RSP_READ_SUPPORTED_DEVICES[RSP_LENGTHS[CMD_IF_READ_SUPPORTED_DEVICES]] = {0x44};

/** Command: Select device. */
static const uint8_t CMD_SELECT_DEVICE[CMD_LENGTHS[CMD_ID_SELECT_DEVICE]] = {'T', 0x44};

/** Response: Select device. */
static const uint8_t RSP_SELECT_DEVICE[RSP_LENGTHS[CMD_ID_SELECT_DEVICE]] = {RSP_OK};

/** Command: Enter programming mode. */
static const uint8_t CMD_ENTER_PROG_MODE[CMD_LENGTHS[CMD_ID_ENTER_PROG_MODE]] = {'P'};

/** Response: Enter programming mode. */
static const uint8_t RSP_ENTER_PROG_MODE[RSP_LENGTHS[CMD_ID_ENTER_PROG_MODE]] = {RSP_OK};

/** Command: Read signature. */
static const uint8_t CMD_READ_SIGNATURE[CMD_LENGTHS[CMD_ID_READ_SIGNATURE]] = {'s'};

/** Response: Read signature. */
static const uint8_t RSP_READ_SIGNATURE[RSP_LENGTHS[CMD_ID_READ_SIGNATURE]] = {0x87, 0x95, 0x1e};

/** Command: Read fuse. */
static const uint8_t CMD_READ_FUSE[CMD_LENGTHS[CMD_ID_READ_FUSE]] = {'F'};

/** Response: Read fuse. */
static const uint8_t RSP_READ_FUSE[RSP_LENGTHS[CMD_ID_READ_FUSE]] = {0xFF};

/** Command: Read high fuse. */
static const uint8_t CMD_READ_HIGH_FUSE[CMD_LENGTHS[CMD_ID_READ_HIGH_FUSE]] = {'N'};

/** Response: Read high fuse. */
static const uint8_t RSP_READ_HIGH_FUSE[RSP_LENGTHS[CMD_ID_READ_HIGH_FUSE]] = {0xD0};

/** Command: Read extended fuse. */
static const uint8_t CMD_READ_EXT_FUSE[CMD_LENGTHS[CMD_ID_READ_EXT_FUSE]] = {'Q'};

/** Response: Read extended fuse. */
static const uint8_t RSP_READ_EXT_FUSE[RSP_LENGTHS[CMD_ID_READ_EXT_FUSE]] = {0xC8};

/** Command: Set address. */
static const uint8_t CMD_SET_ADDRESS[CMD_LENGTHS[CMD_ID_SET_ADDRESS]] = {'A'};

/** Response: Set address. */
static const uint8_t RSP_SET_ADDRESS[RSP_LENGTHS[CMD_ID_SET_ADDRESS]] = {RSP_OK};

/** Command: Write flash. */
static const uint8_t CMD_WRITE_FLASH[CMD_LENGTHS[CMD_ID_WRITE_FLASH]] = {'B', 0x00, 0x80, 'F'};

/** Response: Write flash. */
static const uint8_t RSP_WRITE_FLASH[RSP_LENGTHS[CMD_ID_WRITE_FLASH]] = {RSP_OK};

/** Command: Read flash. */
static const uint8_t CMD_READ_FLASH[CMD_LENGTHS[CMD_ID_READ_FLASH]] = {'g', 0x00, 0x80, 'F'};

/** Command: Leave programming mode. */
static const uint8_t CMD_LEAVE_PROG_MODE[CMD_LENGTHS[CMD_ID_LEAVE_PROG_MODE]] = {'L'};

/** Response: Leave programming mode. */
static const uint8_t RSP_LEAVE_PROG_MODE[RSP_LENGTHS[CMD_ID_LEAVE_PROG_MODE]] = {RSP_OK};

/** Command: Leave bootloader. */
static const uint8_t CMD_LEAVE_BOOTLOADER[CMD_LENGTHS[CMD_ID_LEAVE_BOOTLOADER]] = {'E'};

/** Response: Leave bootloader. */
static const uint8_t RSP_LEAVE_BOOTLOADER[RSP_LENGTHS[CMD_ID_LEAVE_BOOTLOADER]] = {RSP_OK};

/** Commands */
static const uint8_t* COMMANDS[CMD_ID_END] = {
    CMD_READ_SW_ID,          CMD_READ_SW_VERSION,     CMD_READ_PROGRAMMER_TYPE,
    CMD_CHECK_AUTOINCREMENT, CMD_CHECK_BLOCK_SUPPORT, CMD_READ_SUPPORTED_DEVICES,
    CMD_SELECT_DEVICE,       CMD_ENTER_PROG_MODE,     CMD_READ_SIGNATURE,
    CMD_READ_FUSE,           CMD_READ_HIGH_FUSE,      CMD_READ_EXT_FUSE,
    CMD_SET_ADDRESS,         CMD_WRITE_FLASH,         CMD_READ_FLASH,
    CMD_LEAVE_PROG_MODE,     CMD_LEAVE_BOOTLOADER};

/** Responses */
static const uint8_t* RESPONSES[CMD_ID_END] = {
    RSP_READ_SW_ID,          RSP_READ_SW_VERSION,     RSP_READ_PROGRAMMER_TYPE,
    RSP_CHECK_AUTOINCREMENT, RSP_CHECK_BLOCK_SUPPORT, RSP_READ_SUPPORTED_DEVICES,
    RSP_SELECT_DEVICE,       RSP_ENTER_PROG_MODE,     RSP_READ_SIGNATURE,
    RSP_READ_FUSE,           RSP_READ_HIGH_FUSE,      RSP_READ_EXT_FUSE,
    RSP_SET_ADDRESS,         RSP_WRITE_FLASH,         nullptr,
    RSP_LEAVE_PROG_MODE,     RSP_LEAVE_BOOTLOADER};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* CATERINA_SPECIFICATION_H */
/** @} */
