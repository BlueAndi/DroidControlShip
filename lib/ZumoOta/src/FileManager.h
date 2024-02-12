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
#ifndef FILEMANAGER_H
#define FILEMANAGER_H

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <Arduino.h>
/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * File Manager Class.
 */
class FileManager
{
private:
    /**
     *Statische Variable zur Speicherung der aktuellen Dateiposition
     */
     static uint8_t m_filePosition;
public:
    /**
     * Construct the FileManager.
     */
    FileManager();

    /**
     * Destroy the FileManager.
     */
    ~FileManager();

    /**
     * Initializes the FileSystem
     */
    bool init();

    /**
     * Reads 128 bytes from a firmware file stored in LittleFS and stores them into a buffer. 
     * @param firmwareName The name of the firmware file to read.
     * @param buffer       The buffer to store the read bytes. 
     * @return The number of bytes read, or -1 if an error occurred.
     */
    size_t read128Bytes(const char* firmwareName, uint8_t* buffer);
};

#endif /* FileManager_H */
/** @} */
