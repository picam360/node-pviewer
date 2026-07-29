
#include "secrets.h"

#include <Arduino.h>
#include <NimBLEDevice.h>

#ifdef USE_CAT_M
#define TINY_GSM_MODEM_SIM7080
#define TINY_GSM_DEBUG USBSerial
#define TINY_GSM_RX_BUFFER 1024
#include <TinyGsmClient.h>
#else
#include <HTTPClient.h>
#ifdef USE_AWS
#include <WiFiClientSecure.h>
#else
#include <WiFi.h>
#endif
#endif

#include <MQTTClient.h>
#include <ArduinoJson.h>

#include <unordered_map>

// #define TARGET_DEVICE_ATOMS3
#define TARGET_DEVICE_M5DINMETER
#if defined(TARGET_DEVICE_ATOMS3)
#include <M5AtomS3.h> // ATOMS3用ライブラリ
#elif defined(TARGET_DEVICE_M5DINMETER)
#include <M5DinMeter.h>
#define PWR_CTR_PIN 15
#endif

// debug flgs

// network
#ifdef USE_CAT_M
HardwareSerial SerialAT(1);

// SIMモジュールピン（環境に合わせて変更）
#define MODEM_TX 2
#define MODEM_RX 1

TinyGsm modem(SerialAT);
TinyGsmClient net(modem);
#else
#ifdef USE_AWS
WiFiClientSecure net = WiFiClientSecure();
#else
WiFiClient net;
#endif
#endif
MQTTClient client = MQTTClient(256);

// params
static bool g_pwr_ctl = false;
static int g_bat_soc = 0;
static float g_bat_temp = 0;

// UUIDの設定
static BLEUUID LT_BAT_SERVICE_UUID((uint16_t)0xFFE0);
static BLEUUID LT_BAT_READ_UUID((uint16_t)0xFFE1);
static BLEUUID LT_BAT_WRITE_UUID((uint16_t)0xFFE2);

// Renogy BLEモジュールのUUID定義
#define RENOGY_SERVICE_RX_UUID "0000fff0-0000-1000-8000-00805f9b34fb" // 受信・Notify用
#define RENOGY_CHAR_RX_UUID    "0000fff1-0000-1000-8000-00805f9b34fb"
#define RENOGY_SERVICE_TX_UUID "0000ffd0-0000-1000-8000-00805f9b34fb" // 送信・Write用
#define RENOGY_CHAR_TX_UUID    "0000ffd1-0000-1000-8000-00805f9b34fb"

// 送信コマンド (QUERY_BATTERY_STATUS)
const uint8_t LT_BAT_QUERY_STATUS_CMD[] = {0x00, 0x00, 0x04, 0x01, 0x13, 0x55, 0xAA, 0x17};
const uint8_t LT_BAT_DISCHARGE_OFF_CMD[] = {0x00, 0x00, 0x04, 0x01, 0x0d, 0x55, 0xAA, 0x11};

struct BleDeviceInfo {
    char name[64];
    NimBLEAddress addr;
    NimBLEClient *pClient;
    NimBLERemoteCharacteristic *pWriteChar;
    bool doConnect;
    bool connected;
};
static QueueHandle_t bleQueue;
static BleDeviceInfo advDevice_bat = {};
static BleDeviceInfo advDevice_chg = {};

static std::vector<uint8_t> _read_line;
static std::string _ssid = "ERROR_NO_RESPONSE";
static std::string _ip_address = "ERROR_NO_RESPONSE";

// system
static SemaphoreHandle_t serialMutex;
static int status_loop_count = 0;
static unsigned long last_status_msec = 0;
static long status_interval_msec = 100;

// #define DBG_OUT_ENABLE
#define DBGP_BUFF_SIZE 512
static char g_last_dbgp_msg[DBGP_BUFF_SIZE];
void dbgPrintf(char *format, ...)
{
#ifdef DBG_OUT_ENABLE
    if (USBSerial)
    {
        char buff[DBGP_BUFF_SIZE];
        va_list args;
        va_start(args, format);
        vsnprintf(buff, DBGP_BUFF_SIZE, format, args);
        va_end(args);
        memcpy(g_last_dbgp_msg, buff, DBGP_BUFF_SIZE);
        USBSerial.print(buff);
    }
#endif
}

void dbgPrintf(String msg) { dbgPrintf("%s", msg.c_str()); }

// 負荷をON/OFFする関数 (引数に true を渡すとON、false でOFF)
void setPwrCtl(bool turnOn) 
{

    //digitalWrite(PWR_CTR_PIN, turnOn ? HIGH : LOW);

    if (advDevice_chg.pWriteChar == nullptr) {
        USBSerial.println("[エラー] TXキャラスティックが準備されていません");
        return;
    }

    // [ID] [Func=0x06] [Reg_H] [Reg_L] [Data_H] [Data_L] [CRC_L] [CRC_H]
    uint8_t cmdOn[]  = {0x01, 0x06, 0x01, 0x0A, 0x00, 0x01, 0x69, 0xF4};
    uint8_t cmdOff[] = {0x01, 0x06, 0x01, 0x0A, 0x00, 0x00, 0xA8, 0x34};

    // 送信するコマンドを選択
    uint8_t* targetCmd = turnOn ? cmdOn : cmdOff;
    size_t cmdSize = turnOn ? sizeof(cmdOn) : sizeof(cmdOff);

    // 通信方式を自動判定して送信
    if (advDevice_chg.pWriteChar->canWriteNoResponse()) {
        advDevice_chg.pWriteChar->writeValue(targetCmd, cmdSize, false);
    } else {
        advDevice_chg.pWriteChar->writeValue(targetCmd, cmdSize, true);
    }

    if (turnOn) {
        USBSerial.println("[BLE] 負荷を【ON】にするコマンドを送信しました");
    } else {
        USBSerial.println("[BLE] 負荷を【OFF】にするコマンドを送信しました");
    }
}

/** >>>> AWS */
#ifdef USE_CAT_M
String getResponce(int wait_ms)
{
    String responce;
    unsigned long st = millis();
    while (millis() - st < wait_ms)
    {
        while (SerialAT.available())
        {
            char c = (char)SerialAT.read();
            if (c == '\n')
            {
                responce += "\\n";
            }
            else if (c == '\r')
            {
                responce += "\\r";
            }
            else
            {
                responce += c;
            }
        }
    }
    return responce;
}
bool waitModemStable()
{

    return false;
}
void connectCATM()
{
    unsigned long start = millis();

    int step = 0;
    while (millis() - start < 3600000)
    {
        M5.Display.fillScreen(BLACK); // 画面を黒でクリア
        M5.Display.setTextSize(1);    // 文字サイズ設定
        M5.Display.setCursor(0, 0);   // 左上にカーソルセット
        M5.Display.println("Connect CAT-M...");
        M5.Display.print("STEP: ");
        M5.Display.println(step);

        if (step == 0)
        {
            if (!modem.init()){
                step++;
                continue;
            }
            if (!modem.waitForNetwork()){
                step++;
                continue;
            }
            if (!modem.isNetworkConnected())
            {
                step++;
                continue;
            }
            if (!modem.gprsConnect(CATM_APN, CATM_USR, CATM_PWD))
            {
                step++;
                continue;
            }

            M5.Display.println("APN connected");
            delay(1000);
            return;
        }
        if (step == 1)
        {
            M5.Display.println("Restart Modem");
            modem.restart();
            step++;
            delay(1000);
        }
        else if (step == 2)
        {
            String mi = modem.getModemInfo();
            M5.Display.print("Modem: ");
            if (mi.isEmpty())
            {
                M5.Display.println("Wait Info");
                delay(1000);
                continue;
            }
            M5.Display.println(mi);
            step++;
            delay(1000);
        }
        else if (step == 3)
        {
            if (modem.getSimStatus() != 1)
            {
                M5.Display.println("Sim: Not READY");
                delay(1000);
                continue;
            }

            M5.Display.println("Sim: READY");
            step++;
            delay(1000);
        }
        else if (step == 4)
        {

            M5.Display.print("COPS: ");
            SerialAT.println("AT+COPS?");
            String cops = getResponce(1000);
            M5.Display.println(cops);

            int copsValue = cops.substring(cops.indexOf(':') + 1).toInt();
            if (copsValue != 0)
            {
                M5.Display.print("COPS: from ");
                M5.Display.print(copsValue);
                M5.Display.println(" to 0");

                modem.sendAT("+COPS=0");
                modem.waitResponse();
                delay(3000);
            }

            M5.Display.print("CGDCONT: ");
            SerialAT.println("AT+CGDCONT?");
            String cgdcont = getResponce(1000);
            M5.Display.println(cgdcont);

            if (cgdcont.indexOf("\"" CATM_APN "\"") == -1)
            {
                M5.Display.println("APN: " CATM_APN);
                modem.sendAT("+CGDCONT=1,\"IP\",\"" CATM_APN "\"");
                modem.waitResponse();
                delay(3000);
            }

            // M5.Display.print("COPS: ");
            // SerialAT.println("AT+COPS=?");
            // while(!SerialAT.available()){
            //     delay(1000);
            //     M5.Display.print(".");
            // }
            // String copsq = getResponce(1000);
            // M5.Display.println(copsq);
            // USBSerial.println(copsq);

            // M5.Display.print("SET COPS: ");

            // //SerialAT.println("AT+COPS=1,2,\"44020\",7");//softbank
            // SerialAT.println("AT+COPS=1,2,\"44020\"");//softbank
            // while(!SerialAT.available()){
            //     delay(1000);
            //     M5.Display.print(".");
            // }
            // String set_cops = getResponce(1000);
            // M5.Display.print(set_cops);
            // USBSerial.println(set_cops);

            step++;
            delay(1000);
        }
        else if (step == 5)
        {
            M5.Display.println("FIX: CAT-M, LTE");

            M5.Display.print("CMNB: ");
            SerialAT.println("AT+CMNB?");
            String cmnb = getResponce(1000);
            M5.Display.println(cmnb);

            M5.Display.print("CNMP: ");
            SerialAT.println("AT+CNMP?");
            String cnmp = getResponce(1000);
            M5.Display.println(cnmp);

            int cmnbValue = cmnb.substring(cmnb.indexOf(':') + 1).toInt();
            int cnmpValue = cnmp.substring(cnmp.indexOf(':') + 1).toInt();
            if (cmnbValue != 1 || cnmpValue != 38)
            {

                M5.Display.print("CMNB: from ");
                M5.Display.print(cmnbValue);
                M5.Display.println(" to 1");

                M5.Display.print("CNMP: from ");
                M5.Display.print(cnmpValue);
                M5.Display.println(" to 38");

                modem.sendAT("+CFUN=0");
                modem.waitResponse();
                delay(3000);

                modem.sendAT("+CMNB=1");
                modem.waitResponse();
                delay(3000);

                modem.sendAT("+CNMP=38");
                modem.waitResponse();
                delay(3000);

                modem.sendAT("+CFUN=1");
                modem.waitResponse();
                delay(3000);
            }

            step++;
            delay(1000);
        }
        else if (step == 6)
        {
            // ④ 信号確認
            int16_t sq = modem.getSignalQuality();
            M5.Display.print("Signal: ");
            if (sq == 99)
            {
                M5.Display.println("Not Ready");

                M5.Display.print("CSQ: ");
                SerialAT.println("AT+CSQ");
                String csq = getResponce(1000);
                M5.Display.println(csq);

                M5.Display.print("CPIN: ");
                SerialAT.println("AT+CPIN?");
                String cpin = getResponce(1000);
                M5.Display.println(cpin);

                delay(1000);
                continue;
            }
            M5.Display.println(sq);
            M5.Display.println("Modem: Stable!");

            step++;
            delay(1000);
        }
        else if (step == 7)
        {
            M5.Display.print("waitForNetwork: ");
            if (!modem.waitForNetwork(3000))
            {
                M5.Display.println("Failed");

                M5.Display.print("CSQ: ");
                SerialAT.println("AT+CSQ");
                String csq = getResponce(1000);
                M5.Display.println(csq);

                M5.Display.print("CREG: ");
                SerialAT.println("AT+CREG?");
                String creg = getResponce(1000);
                M5.Display.println(creg);

                M5.Display.print("CEREG: ");
                SerialAT.println("AT+CEREG?");
                String cereg = getResponce(1000);
                M5.Display.println(cereg);

                M5.Display.print("CPSI: ");
                SerialAT.println("AT+CPSI?");
                String cpsi = getResponce(1000);
                M5.Display.println(cpsi);

                M5.Display.print("CMNB: ");
                SerialAT.println("AT+CMNB?");
                String cmnb = getResponce(1000);
                M5.Display.println(cmnb);

                M5.Display.print("CNMP: ");
                SerialAT.println("AT+CNMP?");
                String cnmp = getResponce(1000);
                M5.Display.println(cnmp);

                M5.Display.print("COPS: ");
                SerialAT.println("AT+COPS?");
                String cops = getResponce(1000);
                M5.Display.println(cops);

                M5.Display.print("CEER: ");
                SerialAT.println("AT+CEER");
                String ceer = getResponce(1000);
                M5.Display.println(ceer);

                delay(1000);
                continue;
            }

            if (!modem.isNetworkConnected())
            {
                M5.Display.println("Network not connected");
                delay(1000);
                continue;
            }

            M5.Display.println("Network connected");

            step++;
            delay(1000);
        }
        else if (step == 8)
        {
            M5.Display.println("Connecting to APN...");

            if (!modem.gprsConnect(CATM_APN, CATM_USR, CATM_PWD))
            {
                M5.Display.print("CSQ: ");
                SerialAT.println("AT+CSQ");
                String csq = getResponce(1000);
                M5.Display.println(csq);

                delay(1000);
                continue;
            }
            M5.Display.println("APN connected");

            step++;
            delay(1000);
        }
        if (step == 9)
        {
            delay(1000);
            M5.Display.fillScreen(BLACK);
            return;
        }
    }
}
#else
void connectWifi()
{
    M5.Display.fillScreen(BLACK); // 画面を黒でクリア
    M5.Display.setTextSize(1);    // 文字サイズ設定
    M5.Display.setCursor(0, 0);   // 左上にカーソルセット
    M5.Display.println("Connecting...");
    M5.Display.println("Wi-Fi...");

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        M5.Display.print("."); // 画面にドットを追加していく
        USBSerial.print(".");  // シリアルにも出力
    }

    // 接続完了の表示
    M5.Display.println("\nOK!");
    delay(2000);                  // メッセージを確認するために少し待機
    M5.Display.fillScreen(BLACK); // 画面をクリアしてメイン処理へ
}
#endif
#ifdef USE_AWS
void messageHandler(String &topic, String &payload)
{
    USBSerial.println("Topic: " + topic);
    USBSerial.println("Payload: " + payload);

    // JSON解析
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload);

    if (error)
    {
        USBSerial.println("JSON parse failed");
        return;
    }

    const bool pwr_ctl = doc["pwr_ctl"];
    g_pwr_ctl = pwr_ctl;
    setPwrCtl(g_pwr_ctl);

    USBSerial.println("DBG : mqtt subscribed");
}
void connectAWS()
{
    M5.Display.fillScreen(BLACK); // 画面を黒でクリア
    M5.Display.setTextSize(1);    // 文字サイズ設定
    M5.Display.setCursor(0, 0);   // 左上にカーソルセット
    M5.Display.println("Connecting...");
    M5.Display.println("AWS IoT...");

    net.setCACert(AWS_CERT_CA);
    net.setCertificate(AWS_CERT_CRT);
    net.setPrivateKey(AWS_CERT_PRIVATE);

    client.onMessage(messageHandler); // コールバックをセット
    client.begin(AWS_IOT_ENDPOINT, 8883, net);

    // 接続試行中も表示を更新
    while (!client.connect(THINGNAME))
    {
        delay(1000);
        M5.Display.print(".");
        USBSerial.print(".");
    }
    M5.Display.println("Subscribe");
    M5.Display.println(String(AWS_IOT_SUBSCRIBE_TOPIC));
    bool subret = client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
    M5.Display.println(subret ? "OK!" : "FAILED!");
    delay(2000); // メッセージを確認するために少し待機

    // 接続成功
    M5.Display.fillScreen(BLACK);
    M5.Display.setCursor(0, 0);
    M5.Display.setTextColor(GREEN); // 成功時は緑に
    M5.Display.println("Connected to");
    M5.Display.println("AWS IoT!");

    delay(2000);                  // メッセージを確認するために少し待機
    M5.Display.fillScreen(BLACK); // 画面をクリアしてメイン処理へ
}
#else
void messageHandler(String &topic, String &payload)
{
    USBSerial.println("Topic: " + topic);
    USBSerial.println("Payload: " + payload);

    // 1. トピックがRPCリクエストか確認
    String rpcRequestTopic = "v1/devices/me/rpc/request/";
    if (!topic.startsWith(rpcRequestTopic))
    {
        return; // RPC以外のトピックは無視
    }

    // 2. トピックの末尾から Request ID を抽出
    String requestId = topic.substring(rpcRequestTopic.length());

    // 3. JSON解析
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload);

    if (error)
    {
        USBSerial.println("JSON parse failed");
        return;
    }

    // 4. メソッド名の確認（ThingsBoardのウィジェット側で指定したmethod名。例: "setPwrCtl"）
    const char *method = doc["method"];

    if (method && strcmp(method, "set_pwr_ctl") == 0)
    {
        // params の中身を取得（単一の値、またはオブジェクト）
        // ThingsBoardのスイッチの設定次第で `doc["params"]` が直接 boolean だったり、オブジェクトだったりします
        bool pwr_ctl = doc["params"];

        g_pwr_ctl = pwr_ctl;

        setPwrCtl(g_pwr_ctl);
        USBSerial.println("DBG : GPIO State Changed via RPC");

        // 5. サーバー（ダッシュボード）へレスポンスを返却
        // レスポンスを返さないと、ダッシュボード側で「タイムアウトエラー」になります
        String responseTopic = "v1/devices/me/rpc/response/" + requestId;

        JsonDocument responseDoc;
        responseDoc["success"] = true; // クライアント側に返すステータス
        responseDoc["pwr_ctl"] = g_pwr_ctl;

        String responsePayload;
        serializeJson(responseDoc, responsePayload);

        // MQTTでレスポンスをPublish
        client.publish(responseTopic.c_str(), responsePayload.c_str());
        USBSerial.println("DBG : Sent RPC response to " + responseTopic);
    }
    else
    {
        USBSerial.println("Unknown RPC method received");
    }
}
void connectTB()
{
    M5.Display.fillScreen(BLACK); // 画面を黒でクリア
    M5.Display.setTextSize(1);    // 文字サイズ設定
    M5.Display.setCursor(0, 0);   // 左上にカーソルセット
    M5.Display.println("Connecting...");
    M5.Display.println("ThingsBoard...");

    client.onMessage(messageHandler);
    client.begin(TB_SERVER, TB_PORT, net);

    {
        unsigned long startAttemptTime = millis(); // 接続開始時間を記録
        const unsigned long TIMEOUT_MS = 60000;    // タイムアウト時間を1分(60000ミリ秒)に設定
        while (!client.connect(THINGNAME, TB_TOKEN, ""))
        {
            if (millis() - startAttemptTime >= TIMEOUT_MS) {
                USBSerial.println("\nConnection timeout! Rebooting...");
                M5.Display.fillScreen(BLACK);
                M5.Display.setCursor(0, 0);
                M5.Display.setTextColor(RED); // 成功時は緑に
                M5.Display.println("Timeout. Rebooting...");
                delay(5000); // 画面やシリアルに文字を出力し切るための少しの猶予
                ESP.restart(); // システム再起動
            }
            delay(1000);
            M5.Display.print(".");
            USBSerial.print(".");
        }
    }

    // 接続成功
    M5.Display.fillScreen(BLACK);
    M5.Display.setCursor(0, 0);
    M5.Display.setTextColor(GREEN); // 成功時は緑に
    M5.Display.println("Connected to");
    M5.Display.println("ThingBoard!");

    M5.Display.println("Subscribe: " TB_SUBSCRIBE_TOPIC);
    bool subret = client.subscribe(TB_SUBSCRIBE_TOPIC);
    if (subret)
    {
        M5.Display.println("OK!");
    }
    else
    {
        M5.Display.setTextColor(RED); // 成功時は緑に
        M5.Display.println("FAILED!");
        delay(2000);
    }

    delay(1000);
    M5.Display.fillScreen(BLACK); // 画面をクリアしてメイン処理へ
}
#endif
/** <<<< AWS */

/** >>>> BLE */
// Modbus RTU CRC-16 計算関数
uint16_t calculateModbusCRC(const uint8_t *data, uint8_t len) 
{
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 1) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}
uint8_t calcLiTimeChecksum(const uint8_t* data, size_t length) {
uint16_t sum = 0;
  for (size_t i = 0; i < length; i++) {
    sum += data[i];
  }
  return (uint8_t)(sum & 0xFF);
}
// エンディアン変換用（Pythonのrev_hexをシミュレート）
uint32_t get_uint32_le(const uint8_t *data, int start)
{
    return (data[start + 3] << 24) | (data[start + 2] << 16) | (data[start + 1] << 8) | data[start];
}

uint16_t get_uint16_le(const uint8_t *data, int start)
{
    return (data[start + 1] << 8) | data[start];
}

// データパースと画面表示
void parse_litime(const uint8_t *data, size_t length)
{
    if (length < 90)
        return; // 最低限必要なデータ長をチェック

    // 電圧・電流・容量の解析
    float total_voltage = get_uint32_le(data, 8) / 1000.0;

    int16_t raw_current = get_uint16_le(data, 48);
    // Pythonの「r = ~raw_current; (-r if r > 0 else raw_current)」に相当する符号付き処理
    float current = ((int16_t)raw_current) / 1000.0;

    int16_t cell_temp_raw = get_uint16_le(data, 52);
    float cell_temp = (float)cell_temp_raw; // 2の補数処理はint16_tのキャストで自動適用されます

    int soc = data[90]; // 90番目のバイト

    // // 画面の更新
    // AtomS3.Display.clear();
    // AtomS3.Display.setCursor(0, 10);
    // AtomS3.Display.printf("SOC: %d%%\n", soc);
    // AtomS3.Display.printf("Volt: %.2fV\n", total_voltage);
    // AtomS3.Display.printf("Curr: %.2fA\n", current);
    // AtomS3.Display.printf("Temp: %.1fC\n", cell_temp);

    g_bat_soc = soc;
    g_bat_temp = cell_temp;

    USBSerial.printf("SOC: %d%%, V: %.2fV, A: %.2fA, Temp: %.1fC\n", soc, total_voltage, current, cell_temp);
}

// 通知（Notify）コールバック
static void notifyCallback_bat(NimBLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify)
{
    parse_litime(pData, length);
}

// Renogyからの応答を受け取るコールバック
void notifyCallback_chg(NimBLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify)
{
    // もし 7バイトあり、ID=0x01、Read応答(0x03)、データ長2バイト(0x02) なら
    if (length >= 7 && pData[0] == 0x01 && pData[1] == 0x03 && pData[2] == 0x02) {

        // pData[3] が上位バイト、pData[4] が下位バイト
        uint16_t value = (pData[3] << 8) | pData[4];
        USBSerial.printf("[BLE:read 1 word] 0x%04X\n", value);

        //0x1020
        // 負荷のON/OFFは「下位バイト(pData[4]) の ビット15」に格納されている
        uint8_t loadState = (value >> 15) & 0x01;
        
        if (loadState == 1) {
            g_pwr_ctl = true;
        } else if (loadState == 0) {
            g_pwr_ctl = false;
        }
    } 
    // 書き込み(0x06)に対するエコーバック応答の場合
    else if (length >= 8 && pData[0] == 0x01 && pData[1] == 0x06) {
        USBSerial.println("[Renogy] 書き込み(ON/OFF)コマンドが正常に受理されました");
    }
    else {
        // エラー等のその他の応答
        USBSerial.print("[BLE] 別の応答を受信: ");
        for (size_t i = 0; i < length; i++) {
            USBSerial.printf("%02X ", pData[i]);
        }
        USBSerial.println();
    }
}

// BLEスキャンコールバック
class MyAdvertisedDeviceCallbacks : public NimBLEAdvertisedDeviceCallbacks
{
    void onResult(NimBLEAdvertisedDevice *advertisedDevice)
    {
        BleDeviceInfo devInfo = {};

        strncpy(devInfo.name, advertisedDevice->getName().c_str(), sizeof(devInfo.name) - 1);
        devInfo.name[sizeof(devInfo.name) - 1] = '\0';

        devInfo.addr = advertisedDevice->getAddress();

        xQueueSend(bleQueue, &devInfo, 0);
    }
};

// 接続処理
bool connectToBle_bat()
{
    if (advDevice_bat.pClient == nullptr)
    {
        advDevice_bat.pClient = NimBLEDevice::createClient();
    }

    if (!advDevice_bat.pClient->connect(advDevice_bat.addr))
        return false;

    // {
    //     std::vector<NimBLERemoteService*>* services = pClient->getServices(true);
    //     if(USBSerial){
    //         USBSerial.println("\n========= 【重要】発見されたサービスUUID一覧 =========");
    //     }
    //     for (auto* service : *services) {
    //         String uuidStr = service->getUUID().toString().c_str();
    //         if(USBSerial){
    //             USBSerial.printf(" 🔍 発見: %s\n", uuidStr.c_str());
    //         }
    //     }
    //     if(USBSerial){
    //         USBSerial.println("====================================================\n");
    //     }
    // }

    NimBLERemoteService *pRemoteService = advDevice_bat.pClient->getService(LT_BAT_SERVICE_UUID);

    if (pRemoteService == nullptr)
    {
        USBSerial.println("[エラー] サービス(0xFFE0)が見つかりませんでした。");
        advDevice_bat.pClient->disconnect();
        return false;
    }
    USBSerial.println("[BLE] サービス(0xFFE0)の特定に成功！");

    // {
    //     std::vector<NimBLERemoteCharacteristic*>* characteristics = pRemoteService->getCharacteristics(true);
    //     if(USBSerial){
    //         USBSerial.println("\n========= 【重要】発見されたcharacteristic UUID一覧 =========");
    //     }
    //     for (auto* characteristic : *characteristics) {
    //         String uuidStr = characteristic->getUUID().toString().c_str();
    //         if(USBSerial){
    //             USBSerial.printf(" 🔍 発見: %s\n", uuidStr.c_str());
    //         }
    //     }
    //     if(USBSerial){
    //         USBSerial.println("====================================================\n");
    //     }
    // }

    NimBLERemoteCharacteristic *pReadChar = pRemoteService->getCharacteristic(LT_BAT_READ_UUID);
    if (pReadChar && pReadChar->canNotify())
    {
        pReadChar->subscribe(true, notifyCallback_bat);
        USBSerial.println("[BLE] Notify（通知）の登録完了！");
    }
    else
    {
        USBSerial.println("[エラー] READキャラスティックが見つからない、またはNotifyに対応していません。");
        advDevice_bat.pClient->disconnect();
        return false;
    }

    advDevice_bat.pWriteChar = pRemoteService->getCharacteristic(LT_BAT_WRITE_UUID);
    if (advDevice_bat.pWriteChar == nullptr)
    {
        USBSerial.println("[エラー] WRITEキャラスティックが見つかりません。");
        advDevice_bat.pClient->disconnect();
        return false;
    }

    USBSerial.println("[BLE] すべての接続・初期化が正常に完了しました！");
    return true;
}
bool connectToBle_chg()
{
    if (advDevice_chg.pClient == nullptr)
    {
        advDevice_chg.pClient = NimBLEDevice::createClient();
    }

    if (!advDevice_chg.pClient->connect(advDevice_chg.addr)) {
        USBSerial.println("[エラー] チャージャーへの接続に失敗しました。");
        return false;
    }

    // --- 1. RX (受信・Notify) サービスの取得 ---
    NimBLERemoteService *pRxService = advDevice_chg.pClient->getService(RENOGY_SERVICE_RX_UUID);
    if (pRxService == nullptr)
    {
        USBSerial.println("[エラー] RXサービス(FFF0)が見つかりませんでした。");
        advDevice_chg.pClient->disconnect();
        return false;
    }

    NimBLERemoteCharacteristic *pReadChar = pRxService->getCharacteristic(RENOGY_CHAR_RX_UUID);
    if (pReadChar && pReadChar->canNotify())
    {
        pReadChar->subscribe(true, notifyCallback_chg);
        USBSerial.println("[CHG] Notify（通知）の登録完了！");
    }
    else
    {
        USBSerial.println("[CHG:エラー] RXキャラスティックが見つからない、またはNotify非対応です。");
        advDevice_chg.pClient->disconnect();
        return false;
    }

    // --- 2. TX (送信・Write) サービスの取得 ---
    NimBLERemoteService *pTxService = advDevice_chg.pClient->getService(RENOGY_SERVICE_TX_UUID);
    if (pTxService == nullptr)
    {
        USBSerial.println("[CHG:エラー] TXサービス(FFD0)が見つかりませんでした。");
        advDevice_chg.pClient->disconnect();
        return false;
    }

    advDevice_chg.pWriteChar = pTxService->getCharacteristic(RENOGY_CHAR_TX_UUID);
    if (advDevice_chg.pWriteChar == nullptr)
    {
        USBSerial.println("[CHG:エラー] TXキャラスティック(FFD1)が見つかりません。");
        advDevice_chg.pClient->disconnect();
        return false;
    }

    USBSerial.println("[CHG] すべての接続・初期化が正常に完了しました！");
    return true;
}

/** BLE <<<< */

void servoTask(void *pvParameters)
{
}
static int32_t g_dial_pos = 0;
void dialTask(void *pvParameters)
{
    while (1)
    {
        int32_t dial_pos = DinMeter.Encoder.read();
        if (dial_pos != g_dial_pos)
        {
            g_dial_pos = dial_pos;
        }
        delay(1);
    }
}

/********************
 * setup
 */
void setup()
{
#if defined(TARGET_DEVICE_ATOMS3)
    auto cfg = M5.config();
    M5.begin(cfg);            // AtomS3初期設定（LCD,UART,I2C,LED）
    M5.Lcd.begin();           // 画面初期化
    M5.Lcd.setRotation(1);    // 画面向き設定（USB位置基準 0：上/ 1：左/ 2：下/ 3：右）
    M5.Lcd.fillScreen(BLACK); // 背景
#elif defined(TARGET_DEVICE_M5DINMETER)
    auto cfg = M5.config();
    DinMeter.begin(cfg, true);
    M5.Lcd.begin();           // 画面初期化
    M5.Lcd.setRotation(1);    // 画面向き設定（USB位置基準 0：上/ 1：左/ 2：下/ 3：右）
    M5.Lcd.fillScreen(BLACK); // 背景

    pinMode(PWR_CTR_PIN, OUTPUT);
    digitalWrite(PWR_CTR_PIN, LOW); // 初期状態はオフ
#endif

    serialMutex = xSemaphoreCreateMutex();
    bleQueue = xQueueCreate(20, sizeof(BleDeviceInfo));

    USBSerial.begin(115200); // need to be called for USBSerial.isPlugged=true
    // USBSerial.setRxBufferSize(4096);//for big rtcm data
    USBSerial.println("DBG : setup started.");

    // aws
#ifdef USE_CAT_M
    SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
    connectCATM();
#else
    connectWifi();
#endif
#ifdef USE_AWS
    connectAWS();
#else
    connectTB();
#endif

    // ble
    NimBLEDevice::init("");
    NimBLEScan *pBLEScan = NimBLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setInterval(45);
    pBLEScan->setWindow(15);
    pBLEScan->setActiveScan(true);
    pBLEScan->start(10, false);

    // xTaskCreatePinnedToCore(
    //     servoTask,  // 実行する関数
    //     "ServoTask", // タスク名
    //     4096, // スタックサイズ
    //     NULL, // パラメータ
    //     1, // 優先度
    //     NULL, // ハンドル
    //     0 // コア0で実行
    // );

    xTaskCreatePinnedToCore(
        dialTask,       // 実行する関数
        "DialoderTask", // タスク名
        4096,           // スタックサイズ
        NULL,           // パラメータ
        1,              // 優先度
        NULL,           // ハンドル
        0               // コア0で実行
    );
}

void LCD_printf(const char *format, ...)
{
    char buff[64];
    va_list args;
    va_start(args, format);
    vsnprintf(buff, 64, format, args);
    va_end(args);

    int x = M5.Lcd.textWidth(buff);
    int y = M5.Lcd.getCursorY();
    int w = DinMeter.Display.width() - x;
    int h = M5.Lcd.fontHeight();

    M5.Lcd.printf("%s", buff);
    M5.Lcd.fillRect(x, y, w, h, BLACK);
}

/********************
 * loop
 */
void loop()
{
    unsigned long msec = millis();

    if (msec - last_status_msec >= status_interval_msec)
    {
        last_status_msec = msec;

#if defined(TARGET_DEVICE_ATOMS3)

        static int display_mode = 0;
        static bool button_state = false;

        M5.update();

        if (M5.BtnA.wasPressed())
        {
            button_state = true;
        }
        else if (M5.BtnA.wasReleased())
        {
            if (button_state)
            {
                display_mode++;

                M5.Lcd.fillScreen(BLACK); // 背景
            }
            button_state = false;
        }

        M5.Lcd.setTextColor(WHITE, BLACK);             // 文字色
        M5.Lcd.setTextFont(2);                         // フォント
        M5.Lcd.setCursor(0, 0);                        // カーソル座標指定
        LCD_printf("SSID:%.11s\n", _ssid.c_str());     // アクセスポイント時のSSID表示
        M5.Lcd.setTextColor(ORANGE, BLACK);            // 文字色
        LCD_printf("IP:%.13s\n", _ip_address.c_str()); // IPアドレス表示
        M5.Lcd.drawFastHLine(0, 34, 128, WHITE);       // 指定座標から横線

        M5.Lcd.setCursor(0, 38);          // カーソル座標指定
        M5.Lcd.setTextColor(CYAN, BLACK); // 文字色
        switch (display_mode % 3)
        {
        case 2:
            LCD_printf("*****\n");
            break;
        case 1:
            LCD_printf("*****\n");
            break;
        case 0:
        default:
            LCD_printf("*****\n");
            break;
        }
#elif defined(TARGET_DEVICE_M5DINMETER)
        static bool long_press = false;

        DinMeter.update();

        // DinMeter.Speaker.tone(8000, 20);
        M5.Lcd.setTextColor(WHITE, BLACK);    // 文字色
        M5.Lcd.setTextSize(1);                // 文字サイズ設定
        M5.Lcd.setTextFont(2);                // フォント
        M5.Lcd.setCursor(0, 0);               // カーソル座標指定
        LCD_printf("ID: %s\n", THINGNAME); // name
        LCD_printf("DIAL: %d\n", g_dial_pos);
        LCD_printf("USB: %s\n", USBSerial ? "1" : "0");
        if (advDevice_chg.connected)
        {
            LCD_printf("PWR: %s\n", g_pwr_ctl ? "ON" : "OFF");
        }
        else
        {
            LCD_printf("PWR: -\n");
        }
        if (advDevice_bat.connected)
        {
            LCD_printf("BAT: %d%%, %.1fC\n", g_bat_soc, g_bat_temp);
        }
        else
        {
            LCD_printf("BAT: -\n");
        }

        // M5.Lcd.drawFastHLine(0, 50, 128, WHITE);           // 指定座標から横線

        // M5.Lcd.setCursor(0, 50);                           // カーソル座標指定

        if (M5.BtnA.wasPressed())
        {
            DinMeter.Encoder.readAndReset();
            // DinMeter.Encoder.write(0);
            USBSerial.println("DBG : wasPressed");
        }
        if (long_press == false && M5.BtnA.pressedFor(3000))
        {
            long_press = true;
            if (advDevice_bat.connected)
            {
                if (advDevice_bat.pClient->isConnected())
                {
                    if (advDevice_bat.pWriteChar != nullptr)
                    {
                        M5.Lcd.setTextColor(RED, BLACK);    // 文字色
                        LCD_printf("Battery discharg off...\n");
                        delay(5000);

                        // uint8_t cmd[8] = {0x00, 0x00, 0x04, 0x01, 0x0d, 0x55, 0xAA, 0x00};
                        // cmd[7] = calcLiTimeChecksum(cmd, 7);
                        // advDevice_bat.pWriteChar->writeValue(cmd, sizeof(cmd), true);
                        advDevice_bat.pWriteChar->writeValue(LT_BAT_DISCHARGE_OFF_CMD, sizeof(LT_BAT_DISCHARGE_OFF_CMD), true);
                    }
                }
            }

            USBSerial.println("DBG : wasLongPressed");
        }
        if (M5.BtnA.wasReleased())
        {
            long_press = false;
        }
#endif

        //     status_loop_count++;
        //     int step_count = 20;
        //     if ((status_loop_count % step_count) == 0)
        //     {
        //         int step = (status_loop_count / step_count) % 2;
        //         switch (step)
        //         {
        //         case 0:
        //             break;
        //         case 1:
        //             break;
        //         }
        //     }
    }

    // aws
    client.loop();
    if (!client.connected())
    {
#ifdef USE_CAT_M
        connectCATM();
#else
        connectWifi();
#endif
#ifdef USE_AWS
        connectAWS();
#else
        connectTB();
#endif
    }

    // 10秒ごとに送信
    static unsigned long lastMillis = 0;
    if (millis() - lastMillis > 10000)
    {
        lastMillis = millis();

        JsonDocument doc; // ArduinoJson v7の書き方
        doc["time"] = millis();
        doc["bat_soc"] = g_bat_soc;
        doc["bat_temp"] = g_bat_temp;
        doc["pwr_ctl"] = g_pwr_ctl ? 1 : 0;

        char jsonBuffer[512];
        serializeJson(doc, jsonBuffer);

#ifdef USE_AWS
        if (client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer))
#else
        if (client.publish(TB_PUBLISH_TOPIC, jsonBuffer))
#endif
        {
            USBSerial.println("Published: " + String(jsonBuffer));
        }
    }

    {
        BleDeviceInfo devInfo = {};
        if (xQueueReceive(bleQueue, &devInfo, 0)) {
            USBSerial.printf("BLE dev found : %s(%s)\n", devInfo.name, devInfo.addr.toString().c_str());
            if (advDevice_bat.name[0] == '\0' && strcmp(devInfo.name, BATTERY_NAME) == 0)
            {
                advDevice_bat = devInfo;
                advDevice_bat.doConnect = true;
            }
            if (advDevice_chg.name[0] == '\0' && strcmp(devInfo.name, CHARGER_NAME) == 0)
            {
                advDevice_chg = devInfo;
                advDevice_chg.doConnect = true;
            }
            if (advDevice_bat.name[0] != '\0' && advDevice_chg.name[0] != '\0') {
                NimBLEDevice::getScan()->stop();
            }
        }
    }
    // ble
    if (!advDevice_bat.connected && advDevice_bat.doConnect)
    {
        advDevice_bat.doConnect = false;
        if (connectToBle_bat())
        {
            advDevice_bat.connected = true;
        }
        else
        {
            memset(&advDevice_bat, 0, sizeof(advDevice_bat));
            delay(2000);
            NimBLEDevice::getScan()->start(10, false); // 再スキャン
        }
    }
    if (!advDevice_chg.connected && advDevice_chg.doConnect)
    {
        advDevice_chg.doConnect = false;
        if (connectToBle_chg())
        {
            advDevice_chg.connected = true;
        }
        else
        {
            memset(&advDevice_chg, 0, sizeof(advDevice_chg));
            delay(2000);
            NimBLEDevice::getScan()->start(10, false); // 再スキャン
        }
    }

    // 接続中なら1秒ごとにリクエストコマンドを送信
    if (advDevice_bat.connected)
    {
        if (advDevice_bat.pClient->isConnected())
        {
            if (advDevice_bat.pWriteChar != nullptr)
            {
                // uint8_t cmd[8] = {0x00, 0x00, 0x04, 0x01, 0x13, 0x55, 0xAA, 0x00};
                // cmd[7] = calcLiTimeChecksum(cmd, 7);
                // advDevice_bat.pWriteChar->writeValue(cmd, sizeof(cmd), true);
                advDevice_bat.pWriteChar->writeValue(LT_BAT_QUERY_STATUS_CMD, sizeof(LT_BAT_QUERY_STATUS_CMD), true);
            }
            delay(1000);
        }
        else
        {
            memset(&advDevice_bat, 0, sizeof(advDevice_bat));
            delay(2000);
            NimBLEDevice::getScan()->start(10, false);
        }
    }
    if (advDevice_chg.connected)
    {
        if (advDevice_chg.pClient->isConnected())
        {
            if (advDevice_chg.pWriteChar != nullptr)
            {
                // レジスタ 0x010A (負荷状態) から 1ワード を読み取るModbusコマンド
                // 構成: [0x01(ID)] [0x03(Read)] [0x01(Addr_H)] [0x0A(Addr_L)] [0x00(Num_H)] [0x01(Num_L)] [0xA4(CRC_L)] [0x36(CRC_H)]
                uint8_t readCmd[] = {0x01, 0x03, 0x01, 0x20, 0x00, 0x01, 0x00, 0x00};
                
                uint16_t crc = calculateModbusCRC(readCmd, 6);
                // ModbusのCRCは リトルエンディアン (下位バイトが先)
                readCmd[6] = crc & 0xFF;        
                readCmd[7] = (crc >> 8) & 0xFF;
                
                advDevice_chg.pWriteChar->writeValue(readCmd, sizeof(readCmd), false);
            }
            delay(1000);
        }
        else
        {
            memset(&advDevice_chg, 0, sizeof(advDevice_chg));
            delay(2000);
            NimBLEDevice::getScan()->start(10, false);
        }
    }

    while (USBSerial && USBSerial.available() > 0)
    {
        int c = USBSerial.read();
        if (c == '\n')
        {
            _read_line.push_back('\0');
            USBSerial.printf("ECH %s\n", (char *)_read_line.data());
            _read_line.clear();
        }
        else if (c == '\r')
        {
            // do nothing
        }
        else
        {
            _read_line.push_back(c);
        }
    }
}