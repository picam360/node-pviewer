
#include <Preferences.h>

struct Config {
    String CATM_APN;
    String CATM_USR;
    String CATM_PWD;

    String WIFI_SSID;
    String WIFI_PASSWORD;
    String PING_IP;

    String THING_NAME;
    String TB_TOKEN;

    String BATTERY_NAME;
    String CHARGER_NAME;
};

Config config;

// ================================
// 設定をNVSから読み込む
// ================================
bool loadConfig()
{
    Preferences prefs;

    if (!prefs.begin("config", false)) {
        USBSerial.println("loadConfig: begin failed");
        return false;
    }

    {
        const String value = prefs.getString("CATM_APN", "");
        USBSerial.printf("CATM_APN: [%s]\n", value.c_str());
        config.CATM_APN = value;
    }
    {
        const String value = prefs.getString("CATM_USR", "");
        USBSerial.printf("CATM_USR: [%s]\n", value.c_str());
        config.CATM_USR = value;
    }
    {
        const String value = prefs.getString("CATM_PWD", "");
        USBSerial.printf("CATM_PWD: [%s]\n", value.c_str());
        config.CATM_PWD = value;
    }

    {
        const String value = prefs.getString("WIFI_SSID", "");
        USBSerial.printf("WIFI_SSID: [%s]\n", value.c_str());
        config.WIFI_SSID = value;
    }
    {
        const String value = prefs.getString("WIFI_PASSWORD", "");
        USBSerial.printf("WIFI_PASSWORD: [%s]\n", value.c_str());
        config.WIFI_PASSWORD = value;
    }

    {
        const String value = prefs.getString("THING_NAME", "");
        USBSerial.printf("THING_NAME: [%s]\n", value.c_str());
        config.THING_NAME = value;
    }
    {
        const String value = prefs.getString("TB_TOKEN", "");
        USBSerial.printf("TB_TOKEN: [%s]\n", value.c_str());
        config.TB_TOKEN = value;
    }

    {
        const String value = prefs.getString("BATTERY_NAME", "");
        USBSerial.printf("BATTERY_NAME: [%s]\n", value.c_str());
        config.BATTERY_NAME = value;
    }
    {
        const String value = prefs.getString("CHARGER_NAME", "");
        USBSerial.printf("CHARGER_NAME: [%s]\n", value.c_str());
        config.CHARGER_NAME = value;
    }

    prefs.end();

    return true;
}
bool loadConfig_from_json(const String& data)
{
    char *json_str = (char*)data.c_str();
    const String prefix = "data:application/json;base64,";

    if (data.startsWith(prefix)) {
        String encoded = data.substring(prefix.length());

        // デコード後の必要サイズを取得
        size_t decodedLen = 0;

        int ret = mbedtls_base64_decode(
            nullptr,
            0,
            &decodedLen,
            (const unsigned char*)encoded.c_str(),
            encoded.length()
        );

        if (ret != MBEDTLS_ERR_BASE64_BUFFER_TOO_SMALL) {
            USBSerial.println("Base64 length error");
            return false;
        }

        // JSON用バッファ
        uint8_t* decoded = new uint8_t[decodedLen + 1];

        // Base64 decode
        ret = mbedtls_base64_decode(
            decoded,
            decodedLen,
            &decodedLen,
            (const unsigned char*)encoded.c_str(),
            encoded.length()
        );

        if (ret != 0) {
            USBSerial.println("Base64 decode error");
            delete[] decoded;
            return false;
        }

        // JSON文字列として終端
        decoded[decodedLen] = '\0';

        json_str = (char*)decoded;
    }

    USBSerial.println("JSON:");
    USBSerial.println((char*)json_str);

    // JSON解析
    JsonDocument doc;

    DeserializationError error =
        deserializeJson(doc, json_str, strlen(json_str));

    if(json_str != data.c_str())
    {
        delete[] json_str;
        json_str = NULL;
    }

    if (error) {
        USBSerial.print("JSON error: ");
        USBSerial.println(error.c_str());

        return false;
    }

    // JSONから値を取得

    if (doc.containsKey("CATM_APN")) {
        const char* value = doc["CATM_APN"];
        config.CATM_APN = value;
    }
    if (doc.containsKey("CATM_USR")) {
        const char* value = doc["CATM_USR"];
        config.CATM_USR = value;
    }
    if (doc.containsKey("CATM_PWD")) {
        const char* value = doc["CATM_PWD"];
        config.CATM_PWD = value;
    }

    if (doc.containsKey("WIFI_SSID")) {
        const char* value = doc["WIFI_SSID"];
        config.WIFI_SSID = value;
    }
    if (doc.containsKey("WIFI_PASSWORD")) {
        const char* value = doc["WIFI_PASSWORD"];
        config.WIFI_PASSWORD = value;
    }

    if (doc.containsKey("THING_NAME")) {
        const char* value = doc["THING_NAME"];
        config.THING_NAME = value;
    }
    if (doc.containsKey("TB_TOKEN")) {
        const char* value = doc["TB_TOKEN"];
        config.TB_TOKEN = value;
    }

    if (doc.containsKey("BATTERY_NAME")) {
        const char* value = doc["BATTERY_NAME"];
        config.BATTERY_NAME = value;
    }
    if (doc.containsKey("CHARGER_NAME")) {
        const char* value = doc["CHARGER_NAME"];
        config.CHARGER_NAME = value;
    }

    return true;
}

// ================================
// 設定をNVSへ保存
// ================================
bool saveConfig()
{
    Preferences prefs;

    if (!prefs.begin("config", false)) {
        USBSerial.println("saveConfig: begin failed");
        return false;
    }

    {
        const String value = config.CATM_APN;
        USBSerial.printf("CATM_APN: [%s]\n", value.c_str());
        prefs.putString("CATM_APN", value);
    }
    {
        const String value = config.CATM_USR;
        USBSerial.printf("CATM_USR: [%s]\n", value.c_str());
        prefs.putString("CATM_USR", value);
    }
    {
        const String value = config.CATM_PWD;
        USBSerial.printf("CATM_PWD: [%s]\n", value.c_str());
        prefs.putString("CATM_PWD", value);
    }

    {
        const String value = config.WIFI_SSID;
        USBSerial.printf("WIFI_SSID: [%s]\n", value.c_str());
        prefs.putString("WIFI_SSID", value);
    }
    {
        const String value = config.WIFI_PASSWORD;
        USBSerial.printf("WIFI_PASSWORD: [%s]\n", value.c_str());
        prefs.putString("WIFI_PASSWORD", value);
    }

    {
        const String value = config.THING_NAME;
        USBSerial.printf("THING_NAME: [%s]\n", value.c_str());
        prefs.putString("THING_NAME", value);
    }
    {
        const String value = config.TB_TOKEN;
        USBSerial.printf("TB_TOKEN: [%s]\n", value.c_str());
        prefs.putString("TB_TOKEN", value);
    }

    {
        const String value = config.BATTERY_NAME;
        USBSerial.printf("BATTERY_NAME: [%s]\n", value.c_str());
        prefs.putString("BATTERY_NAME", value);
    }
    {
        const String value = config.CHARGER_NAME;
        USBSerial.printf("CHARGER_NAME: [%s]\n", value.c_str());
        prefs.putString("CHARGER_NAME", value);
    }

    prefs.end();

    return true;
}