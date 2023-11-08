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
#include <Arduino.h>
#include "ESPAsyncWebServer.h"
#include "Upload.h"

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

class WebServerCustom : public AsyncWebServer{
private:

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
    void begin();
};

#endif/* WEBSERVER_H */
/** @} */

