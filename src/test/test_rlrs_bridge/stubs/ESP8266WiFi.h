#pragma once
enum WiFiMode_t { WIFI_OFF = 0, WIFI_STA = 1, WIFI_AP = 2 };
struct WiFiStub
{
    bool persistentFlag = true; int modeValue = WIFI_AP; bool sleeping = false;
    void persistent(bool p) { persistentFlag = p; }
    bool mode(WiFiMode_t m) { modeValue = m; return true; }
    bool forceSleepBegin() { sleeping = true; return true; }
};
extern WiFiStub WiFi;
