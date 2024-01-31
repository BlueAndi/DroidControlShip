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
     * @param mybytes The size of the expected Response.
     *@return The total number of bytes read from the stream.
     */
    size_t readingStream(uint8_t* expectedResponse,size_t mybytes);

    /**
     * @brief Send a command to the Zumo robot.
     * @param command Array containing the command to be sent.
     * @param commandSize Size of the command array.
     * @return True if the command was successfully sent, otherwise false.
     */
    bool sendCommand(const uint8_t command[], size_t commandSize);

};

#endif /* FlashManager_H */
/** @} */
