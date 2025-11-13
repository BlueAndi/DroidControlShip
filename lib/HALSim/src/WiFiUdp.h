/*
 * Lightweight WiFiUDP stub for native simulation builds.
 * Provides a minimal class to satisfy code that depends on Arduino's WiFiUDP.
 */
#ifndef SIM_WIFIUDP_H
#define SIM_WIFIUDP_H

class WiFiUDP
{
public:
    WiFiUDP()  = default;
    ~WiFiUDP() = default;
};

#endif /* SIM_WIFIUDP_H */

