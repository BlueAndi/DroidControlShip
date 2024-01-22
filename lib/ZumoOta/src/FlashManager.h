/*******************************************************************************
    DESCRIPTION
*******************************************************************************/
/**
 * @brief  ZumoOta application
 * @author Decareme Pauline Ngangnou <ngandeca@yahoo.fr>
 *
 * @addtogroup Application
 *
 * @{
 */
#ifndef FlashManager_H
#define FlashManager_H

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "Zumo32U4Specification.h"
/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * Flash Manager Class.
 */
class FlashManager
{
private:
    /**
     *Number of bytes read from the stream;
     */
     size_t m_bytesRead;


public:
    /**
     * Constructor of  FlashManager.
     */
    FlashManager();

    /**
     * Destructor of  FlashManager.
     */
    ~FlashManager();

    /**
     * @brief Reads a stream of data from the device's input stream.
     * @param expectedResponse Pointer to the buffer to store the expected response.
     *@return The total number of bytes read from the stream.
     */
    size_t readingStream(uint8_t* expectedResponse);

    /**
     * @brief Send a command to the Zumo robot and prepare for receiving a response.
     *
     * @param command Array containing the command to be sent.
     * @param commandSize Size of the command array.
     * @return True if the command was successfully sent, otherwise false.
     */
    bool sendCommand(const uint8_t command[], size_t commandSize);

    /**
     * @brief Checks the response against the expected response after sending a command.
     *
     * @param command The command to be sent.
     * @param commandSize The size of the command.
     * @param expectedResponse The expected response to compare against.
     * @param expectedResponseSize The size of the expected response.
     * @return True if the received response matches the expected response, false otherwise.
     */
    bool Check(const uint8_t command[], size_t commandSize, const uint8_t expectedResponse[], size_t expectedResponseSize);


};

#endif /* FlashManager_H */
/** @} */
