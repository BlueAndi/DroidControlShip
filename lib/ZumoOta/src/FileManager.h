/*******************************************************************************
    DESCRIPTION
*******************************************************************************/
/**
 * @brief  ZumoOta application
 * @author Decareme Pauline Ngangnou <ngandeca@yahoo.fr>
 *
 * 
 *
 * @{
 */
#ifndef FILEMANAGER_H
#define FILEMANAGER_H

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <Arduino.h>
#include <FS.h>
#include <LittleFS.h>
#include <FileReader.h>

/******************************************************************************
 * Macros
 *****************************************************************************/
 /**
 * @brief External declaration of the test_html array stored in program memory.
extern const char test_html[] PROGMEM; 

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

class FileManager {
private:
  
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
        * @brief Creates an upload page.
        * 
        * This function initializes the creation of an upload page and sets up the necessary configurations.
        * 
        * @return Returns true if the upload page creation is successful, false otherwise.
    */
    bool createUploadPage();

    /**
    * @brief Reads the upload page.
        * 
        * This function reads the contents of the upload page and returns it as a String.
        * 
        * @return The content of the upload page as a String.
    */
    String readUploadPage();
};

#endif /* FileManager_H */
/** @} */


