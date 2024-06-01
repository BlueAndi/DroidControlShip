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
 * @brief  Abstraction and Stream implementation of USB Host
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 *
 * @addtogroup HALTarget
 *
 * @{
 */

#ifndef USBHOST_H
#define USBHOST_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <cdcacm.h>
#include <SPI.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** Class for configuring the Abstract Control Model used for the USB CDC (Communications Device Class/Serial) */
class ACMAsyncOper : public CDCAsyncOper
{
public:
    /**
     * Default constructor
     */
    ACMAsyncOper() : CDCAsyncOper(), m_isBootloaderRequested(false)
    {
    }

    /**
     * Default destructor
     */
    ~ACMAsyncOper()
    {
    }

    /**
     * Initializes the CDC device when connected
     *
     * @param[in] pacm The ACM pointer/instance
     * @return Returns result code
     */
    uint8_t OnInit(ACM* pacm);

    /**
     * Activate the bootloader mode.
     */
    void requestBootloaderMode();

private:
    /** Specifies the Control Line State. */
    static const uint8_t CONTROL_LINE_STATE = 1U; /* DTR = 1, RST = 0 */

    /** Specifies the baud rate used for the serial communication */
    static const uint32_t BAUD_RATE_NORMAL_MODE = 115200U;

    /** Specifies the baud rate used for the serial communication */
    static const uint32_t BAUD_RATE_BOOTLOADER_MODE = 57600U;

    /** Specifies the character format */
    static const uint8_t CHAR_FORMAT = 0U;

    /** Specifies parity check configuration */
    static const uint8_t PARITY_TYPE = 0U;

    /** Specifies how many bits are transferred per packet */
    static const uint8_t NUMBER_OF_DATA_BITS = 8U;

    /** Bootloader mode request flag. */
    bool m_isBootloaderRequested;
};

/** Class for configuring and managing the USB Host */
class USBHost : public Stream
{
public:
    /**
     * Default constructor
     */
    USBHost();

    /**
     * Default destructor
     */
    ~USBHost();

    /**
     * Initializes the USB Host.
     *
     * @returns true if initialization is successful. Otherwise, false.
     */
    bool init();

    /**
     * Process the device connection.
     *
     * @returns true if processing is successful. Otherwise, false.
     */
    bool process();

    /**
     * Print single byte.
     * @param[in] value Byte to send.
     * @returns Number of bytes written
     */
    size_t write(uint8_t value) final;

    /**
     * Write bytes.
     * @param[in] buffer Byte Array to send.
     * @param[in] length Length of Buffer.
     * @returns Number of bytes written
     */
    size_t write(const uint8_t* buffer, size_t length) final;

    /**
     * Check if there are available bytes in the Stream.
     * @returns Number of available bytes.
     */
    int available() final;

    /**
     * Read a byte from the Stream.
     * @returns The first byte of incoming data available (or -1 if no data is available).
     */
    int read() final;

    /**
     * Peek a byte from the Stream.
     * @returns The first byte of incoming data available (or -1 if no data is available).
     */
    int peek() final;

    /**
     * Read bytes into a buffer.
     * @param[in] buffer Array to write bytes to.
     * @param[in] length number of bytes to be read.
     * @returns Number of bytes read from Stream.
     */
    size_t readBytes(uint8_t* buffer, size_t length) final;

    /**
     * Request to enter the bootloader mode.
     */
    void requestBootloaderMode();

    /**
     * Reset the flags, ACM and data queue.
     */
    void reset();

    /**
     * Check if the bootloader mode is active.
     *
     * @return If the bootloader mode is active, returns true. Otherwise, false.
     */
    bool isBootloaderModeActive() const;

private:
    /** Size of the RX Queue */
    static const uint16_t USB_RX_QUEUE_SIZE = 1024U;

    /** USB core driver instance for USB Host Shield */
    USB m_usb;

    /** Instance of the CDC initialization class */
    ACMAsyncOper m_asyncOper;

    /** Instance of ACM */
    mutable ACM m_acm;

    /** RX Queue */
    QueueHandle_t m_rxQueue;

    /** Bootloader Mode Flag. */
    bool m_isBootloaderModeActive;

private:
    /**
     * Get a Byte from the receiving buffer, if any.
     * @param[out] byte buffer to write the byte to.
     * @returns If a received byte has been succesfully written to the buffer, returns true. Otherwise, false.
     */
    bool getByte(uint8_t& byte);

private:
    /**
     * Copy construction of an instance.
     * Not allowed.
     *
     * @param[in] usbhost Source instance.
     */
    USBHost(const USBHost& usbhost);

    /**
     * Assignment of an instance.
     * Not allowed.
     *
     * @param[in] usbhost Source instance.
     *
     * @returns Reference to USBHost instance.
     */
    USBHost& operator=(const USBHost& usbhost);
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* USBHOST_H */
/** @} */
