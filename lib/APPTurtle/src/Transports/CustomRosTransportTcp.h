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
 * @brief  Custom Micro-ROS transport using TCP over Arduino WifiClient.
 * @author Norbert Schulz <github@schulznorbert.de>
 *
 * @addtogroup Application
 *
 * @{
 */
#ifndef CUSTOM_ROS_TRANSPORT_TCP_H
#define CUSTOM_ROS_TRANSPORT_TCP_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "CustomRosTransportBase.h"

#include <WiFiClient.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** 
 * Map this transport to the class name used in MicroRosClient.
 * The used transport is a compile time decision and this typedef
 * avoids the use of ifdef's.
 */
typedef class CustomRosTransportTcp CustomRosTransport;

/**
 * Micro-ROS custom transport adaption for TCP.
 *
 * TCP protocol uses a record format with a 16 bit length field before the
 * payload.
 * 
 *  +----------------+----------------+---------------------+
 *  |    1 Byte      |    1 Bytes     |    "Lenght" Bytes   |
 *  +----------------+----------------+---------------------+
 *  | Length [0..7]  | Length [8..15] |      < payload >    |
 *  +----------------+----------------+---------------------+
 * 
 * A state machine is used for reading the record format. 
 * 
 */
class CustomRosTransportTcp : public CustomRosTransportBase
{
public:
    /**
     * Constructs a custom Micro-ROS transport.
     */
    CustomRosTransportTcp() : 
        CustomRosTransportBase(), 
        m_tcpClient(),
        m_inputState(InputState::INIT),
        m_payloadLen(0U),
        m_received(0U),
        m_inputBuf()
    {
    }

    /**
     * Destroys custom Micro-ROS transport.
     *
     */
    virtual ~CustomRosTransportTcp() final
    {
    }

    /** 
     * Get protocol name used by this trandport.
     * @return protocol name
     */
    virtual const String& getProtocolName() const final
    {
        return m_protocolName;
    }

private:
    /**
     * Open and initialize API for the custom transport.
     *
     * @return A boolean indicating if the opening was successful.
     */
    bool open(void) final;

    /**
     * Close API for the custom transport.
     *
     * @return A boolean indicating if the closing was successful.
     */
    bool close(void) final;

    /**
     * Write data API for the custom transport.
     *
     * @param[in]  buffer The buffer to write.
     * @param[in]  size The size of the buffer.
     * @param[out] errorCode The error code.
     *
     * @return The number of bytes written.
     */
    size_t write(const uint8_t* buffer, size_t size, uint8_t* errorCode) final;

    /**
     * Read data API of the custom transport.
     *
     * @param[out] buffer The buffer to read into.
     * @param[in]  size The size of the buffer.
     * @param[in]  timeout The timeout in milliseconds.
     * @param[out] errorCode The error code.
     *
     * @return The number of bytes read.
     */
    size_t read(uint8_t* buffer, size_t size, int timeout, uint8_t* errorCode) final;

    /**
     * Internal read wrapper with timeout handling loop.
     * This function is used by the input state dependend read handlers.
     *
     * @param[out] buffer The buffer to read into.
     * @param[in]  size The size of the buffer.
     * @param[in]  timeout The timeout in milliseconds.
     * @param[out] errorCode Set the error code to != 0 on error.
     *
     * @return The number of bytes read.
     */
    size_t readInternal(uint8_t* buffer, size_t size, int timeout, uint8_t* errorCode);

    /**
     * Try to read 2 byte size prefix (low, high) in InputState::INIT
     * 
     * @param[in]  timeout The timeout in milliseconds.
     * @param[out] errorCode Set the error code to != 0 on error.
     *
     *  @return true if statemachine shall reloop.
     */
    bool readSizePrefix(int timeout, uint8_t* errorCode);

    /**
     * Try to read 2nd byte of size prefix (low, high) in InputState::PREFIX_1
     * 
     * Used in the rare event that low level read only provided one byte.
     * 
     * @param[in]  timeout The timeout in milliseconds.
     * @param[out] errorCode Set the error code to != 0 on error.
     *
     * @return true if statemachine shall reloop.
     */
    bool readPendingSizePrefix(int timeout, uint8_t* errorCode);

    /**
     * Try payload bytes in InputState::PLAY_LOAD
     * 
     * Used in the rare event that low level read only provided one byte.
     * 
     * @param[in]  timeout The timeout in milliseconds.
     * @param[out] errorCode Set the error code to != 0 on error.
     * 
     * @return true if statemachine shall reloop.
     */
    bool readPayload(int timeout, uint8_t* errorCode);

    /**
     * Handle message complete in InputState::FINISH
     * 
     * Used in the rare event that low level read only provided one byte.
     * 
     * @param[in]  timeout The timeout in milliseconds.
     * @param[out] errorCode Set the error code to != 0 on error.
     * 
     * @return true if statemachine shall reloop.
     */
    bool readFinish(int timeout, uint8_t* errorCode);

private:
    WiFiClient            m_tcpClient;     /**< The TCP client.     */
    static const String   m_protocolName;  /**< This protocol name. */
    
    /**
     *  Read buffer handling from streaming sockets to implement framing.
     *
     * Custom transports with framing support need a customized agent.
     * That is why the framing is implemented here to work with the
     * "standard" tcpv4 agent protocol.   
     */
    enum InputState {
        INIT,             /**< Input buffer ready for new record.          */
        PREFIX_1,         /**< First byte of 2 -Byte size prefix received. */
        PLAY_LOAD,        /**< Collecting payload bytes.                   */
        FINISH,           /**< Payload record is complete.                 */
        MAX               /**< Enum Range value, not a true state.         */
    };

    /** State dependent read function pointers 
     *
     * @param[in] timeout    Read timout in milliseconds.
     * @param[out] errorCode Read error code if != 0
     * 
     * return true if state machine shall reloop. 
    */
    typedef bool (CustomRosTransportTcp::*ReadFunc)(int timeout, uint8_t* errorCode);

    /** Read functions ponter array indexed by InputState. */
    static const ReadFunc m_readFunction[MAX];

    InputState   m_inputState;       /**< Actual input buffer status         */

    size_t       m_payloadLen;       /**< Number of bytes to read as payload. */
    size_t       m_received;         /**< Number of bytes received already    */

    static const size_t MTU = 1024U; /**< Maximum supported transmission size.*/
    uint8_t      m_inputBuf[MTU];    /**< Buffer for input record reading.    */
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* CUSTOM_ROS_TRANSPORT_TCP_H */
/** @} */
