#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

void setupWifi();   // Blocking initial connect, called once from setup()
void maintainWifi(); // Non-blocking, throttled reconnect - call every loop()

#endif // WIFI_MANAGER_H
