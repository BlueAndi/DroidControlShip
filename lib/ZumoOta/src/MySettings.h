/*******************************************************************************
    DESCRIPTION
*******************************************************************************/
/**
 * @brief  ZumoOta application
 * @author Decareme Pauline Ngangnou <ngandeca@yahoo.fr>
 * @{
 */
#ifndef MySettings_H
#define MySettings_H

/******************************************************************************
 * Includes
 *****************************************************************************/

/******************************************************************************
 * Macros
 *****************************************************************************/
 
/******************************************************************************
 * Types and Classes
 *****************************************************************************/

class MySettings{
private:
        /**
        * Network SSID. 
        */
        const char* m_wifiSSID;

        /**
        * Network Password. 
        */
        const char* m_wifiPassword;

        /**
        * Access Point SSID. 
        */
        const char* m_apSSID;

        /**
        * Access Point Password. 
        */
        const char* m_apPassword;

        /**
        * Basic Authentication username.
        */
        const char* m_authUsername;

        /**
        * Basic Authentication password.
        */
        const char* m_authPassword;
       
public:
    /**
     * Constructor of  MySettings.
     */
    MySettings();

    /**
     * Destructor of  MySettings.
     */
    ~MySettings();

    /**
     * Retrieves the stored Network SSID.
     */
    const char* getWiFiSSID();

    /**
     * Retrieves the stored Network Password SSID.
     */
    const char* getWiFiPassword();

    /**
     * Retrieves the stored Access Point SSID.
     */
    const char* getapSSID();

    /**
     * Retrieves the stored Access Point Password SSID.
     */
    const char* getapPassword();

    /**
     * Sets the WiFi credentials with provided SSID and password.
     */
    void setWiFiCredentials(const char* ssid, const char* password);

    /**
     * Retrieves the stored Basic Authentication username.
     */
    const char* getAuthUsername();

    /**
     * Retrieves the stored Basic Authentication password.
     */
    const char* getAuthPassword();

    /**
     *  Sets the Basic Authentication credentials with provided username and password.
     */
    void setAuthCredentials(const char* username, const char* password);
};

#endif /* MySettings_H */
/** @} */


