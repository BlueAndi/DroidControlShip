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
#include <ESPAsyncWebServer.h>
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
    * Handles file uploads via the web server.
    * 
    * @param request   The web server request pointer
    * @param filename  The name of the uploaded file
    * @param index     The index of the file block being uploaded
    * @param data      Pointer to the data buffer being uploaded
    * @param len       The length of data being uploaded
    * @param final     Indicates if it's the final block of the file
    */
    void handleFileUpload(AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final);
};

#endif/* Upload_H */
/** @} */
