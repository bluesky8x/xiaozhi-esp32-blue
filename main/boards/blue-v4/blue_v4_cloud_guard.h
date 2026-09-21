#ifndef _BLUE_V4_CLOUD_GUARD_H_
#define _BLUE_V4_CLOUD_GUARD_H_

#include "settings.h"

#include <esp_log.h>
#include <cstring>

// Blue V4 clone of blue-v2's cloud guard: the device must talk to the user's own
// server only, never fall back to the xiaozhi cloud endpoints.
//
// The helper names deliberately match the blue-v2 header (only one board compiles
// at a time) so the core OTA / websocket guards work unchanged.
#define BLUE_V4_CLOUD_GUARD_TAG "BlueV4CloudGuard"

inline bool BlueIsCloudServerUrl(const char* url) {
    if (url == nullptr || url[0] == '\0') {
        return false;
    }
    return strstr(url, "tenclass.net") != nullptr || strstr(url, "xiaozhi.me") != nullptr;
}

#ifndef BLUE_BLOCK_CLOUD_SERVERS
#if defined(BLUE_V4_BLOCK_CLOUD_SERVERS) && BLUE_V4_BLOCK_CLOUD_SERVERS
#define BLUE_BLOCK_CLOUD_SERVERS 1
#else
#define BLUE_BLOCK_CLOUD_SERVERS 0
#endif
#endif

#if BLUE_BLOCK_CLOUD_SERVERS

// Drop stale xiaozhi cloud endpoints left in NVS from older firmware or misconfiguration.
inline void BlueSanitizeStoredServerSettings() {
    Settings wifi("wifi", true);
    const std::string ota = wifi.GetString("ota_url");
    if (BlueIsCloudServerUrl(ota.c_str())) {
        wifi.SetString("ota_url", "");
        ESP_LOGW(BLUE_V4_CLOUD_GUARD_TAG,
                 "Cleared cloud OTA URL from NVS — set yours in the WiFi portal");
    }

    Settings ws("websocket", true);
    const std::string ws_url = ws.GetString("url");
    if (BlueIsCloudServerUrl(ws_url.c_str())) {
        ws.SetString("url", "");
        ESP_LOGW(BLUE_V4_CLOUD_GUARD_TAG, "Cleared cloud websocket URL from NVS");
    }

    Settings mqtt("mqtt", true);
    const std::string endpoint = mqtt.GetString("endpoint");
    if (BlueIsCloudServerUrl(endpoint.c_str())) {
        mqtt.SetString("endpoint", "");
        ESP_LOGW(BLUE_V4_CLOUD_GUARD_TAG, "Cleared cloud MQTT endpoint from NVS");
    }
}

#endif  // BLUE_V4_BLOCK_CLOUD_SERVERS

#endif  // _BLUE_V4_CLOUD_GUARD_H_
