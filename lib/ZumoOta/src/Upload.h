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

#ifndef UPLOAD_H
#define UPLOAD_H

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <stdint.h>
#include <Arduino.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

class Upload {
private:
    
public:
    /**
     * Construct the Upload class.
     */
    Upload();

    /**
     * Destruct the Upload class.
     */
    ~Upload();

    /**
        * @brief Handles the upload button press.
        * 
        * This function is responsible for handling the upload button press.
        * It performs necessary actions when the upload button is pressed.
    */
    void handleUploadButtonPress();
};

#endif/* Upload_H */
/** @} */

