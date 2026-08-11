#ifndef BLUE_CLOUD_GUARD_H_
#define BLUE_CLOUD_GUARD_H_

#include "settings.h"

#include <esp_log.h>
#include <cstring>

#define BLUE_CLOUD_GUARD_TAG "BlueCloudGuard"

inline bool BlueIsCloudServerUrl(const char* url) {
    if (url == nullptr || url[0] == '\0') {
        return false;
    }
    return strstr(url, "tenclass.net") != nullptr || strstr(url, "xiaozhi.me") != nullptr;
}

#if BLUE_V2_BLOCK_CLOUD_SERVERS

// Drop stale xiaozhi cloud endpoints left in NVS from older firmware or misconfiguration.
inline void BlueSanitizeStoredServerSettings() {
    Settings wifi("wifi", true);
    const std::string ota = wifi.GetString("ota_url");
    if (BlueIsCloudServerUrl(ota.c_str())) {
        wifi.SetString("ota_url", "");
        ESP_LOGW(BLUE_CLOUD_GUARD_TAG, "Cleared cloud OTA URL from NVS — set yours in WiFi portal");
    }

    Settings ws("websocket", true);
    const std::string ws_url = ws.GetString("url");
    if (BlueIsCloudServerUrl(ws_url.c_str())) {
        ws.SetString("url", "");
        ESP_LOGW(BLUE_CLOUD_GUARD_TAG, "Cleared cloud websocket URL from NVS");
    }

    Settings mqtt("mqtt", true);
    const std::string endpoint = mqtt.GetString("endpoint");
    if (BlueIsCloudServerUrl(endpoint.c_str())) {
        mqtt.SetString("endpoint", "");
        ESP_LOGW(BLUE_CLOUD_GUARD_TAG, "Cleared cloud MQTT endpoint from NVS");
    }
}

#endif  // BLUE_V2_BLOCK_CLOUD_SERVERS

#endif  // BLUE_CLOUD_GUARD_H_
