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
#include "Zumo32U4Specifications.h"
#include "BootloaderCom.h"
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
    /** Specifies the expected firmware size of the firmware to be flashed */
    uint16_t m_expectedFirmwareSize;

    /** Specifies how many bytes have already been written to Zumo program flash */
    uint16_t m_writtenFirmwareBytes;
    
    /** Specifies the expected SHA256 hash string of the firmware to be flashed */
    String m_expectedHashValue;

   
     /**
    * Instance of the BootloaderCom class
    */
    BootloaderCom m_bootloader;

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
     * @brief  Write received firmware data to the robot's flash memory.
     * 
     * This function reads data from the provided stream and writes it to
     * the robot's flash memory.
     */
    void readToRobotFlash(size_t expectedsize);

    /**
     * @brief exit the bootloader mode.
     */
    void exitBootloader();

    /**
     * @brief enter the bootloader mode.
    */
    void enterBootloadermode();

    /**
     * @brief send a Command.
    */
    void sendCommand(const uint8_t command[]);

};

#endif /* FlashManager_H */
/** @} */
