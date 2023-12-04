/*******************************************************************************
    DESCRIPTION
*******************************************************************************/
/**
 * @brief  WebServer realization
 * @author Decareme Pauline Ngangnou <ngandeca@yahoo.fr>
 *
 * @addtogroup Application
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

/**
 * Custom implementation of WebServer.
 */
class WebServerCustom
{
private:
    /**
     * The web server instance.
     */
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

    /**
     * @brief Makes the size of files human-readable.
     *
     * This function converts the file size in bytes to a human-readable format.
     *
     * @param bytes The size of the file in bytes
     * @return A string representing the human-readable file size
     */
    String humanReadableSize(const size_t bytes);

    /**
     * @brief Lists files stored on LittleFS in either HTML or plaintext format.
     *
     * This function generates a list of files stored on LittleFS and returns it as a string.
     * The output format can be either HTML or plaintext, depending on the value of the `ishtml` parameter.
     *
     * @param ishtml Flag indicating whether to generate HTML-formatted output (true) or plaintext (false)
     * @return A string containing the list of files in the specified format
     */
    String listFiles(bool ishtml);
};

#endif /* WEBSERVER_H */
/** @} */
