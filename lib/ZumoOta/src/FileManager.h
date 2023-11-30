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

/******************************************************************************
 * Macros
 *****************************************************************************/
 
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
     * Initializes the FileSystem
     */
    bool init();
};

#endif /* FileManager_H */
/** @} */


