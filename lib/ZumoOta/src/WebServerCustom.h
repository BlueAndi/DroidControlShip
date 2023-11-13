/*******************************************************************************
    DESCRIPTION
*******************************************************************************/
/**
 * @brief  WebServer realization
 * @author Decareme Pauline Ngangnou <ngandeca@yahoo.fr>
 *
 * 
 *
 * @{
 */

#ifndef WebServerCustom_H
#define WebServerCustom_H

/******************************************************************************
 * Includes
 *****************************************************************************/

#include <ESPAsyncWebServer.h>
#include "Upload.h"

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

class WebServerCustom
{
private:
AsyncWebServer server{80};

public:
    /**
     * Construct the WebServerCustom class.
     */
    WebServerCustom();

    /**
     * Destruct the WebServerCustom class.
     */
    ~WebServerCustom();

    /**
        * @brief Initializes the web server.
        * 
        * This function is used to initialize the web server and set it up for handling requests.
    */
    void start();

    /**
        * 
        *This function is used to initialize the web server and set it up for handling requests. 
        * 
    */
    void init();

    /**
        * 
        * This function sets up a handler for processing HTTP POST requests to /upload.
        * 
    */
    void handleUploadRequest();
};

#endif/* WEBSERVER_H */
/** @} */

