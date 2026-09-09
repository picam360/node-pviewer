
#include "secrets.h"

#include <Arduino.h>
#include "mbedtls/base64.h"
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

#include "config.cpp.h"

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
static bool g_pwr_ctl_set_required = false;
static unsigned long g_chg_updated_msec = 0;
static int g_bat_soc = 0;
static float g_bat_temp = 0;
static unsigned long g_bat_updated_msec = 0;

// UUIDの設定
static BLEUUID LT_BAT_SERVICE_UUID((uint16_t)0xFFE0);
static BLEUUID LT_BAT_READ_UUID((uint16_t)0xFFE1);
static BLEUUID LT_BAT_WRITE_UUID((uint16_t)0xFFE2);

// Charger BLEモジュールのUUID定義
//#define LT_CHG
#ifdef LT_CHG

static BLEUUID LT_CHG_SERVICE_UUID((uint16_t)0xFFE0);
static BLEUUID LT_CHG_READ_UUID((uint16_t)0xFFE1);
static BLEUUID LT_CHG_WRITE_UUID((uint16_t)0xFFE2);

#else//Renogy

#define RENOGY_SERVICE_RX_UUID "0000fff0-0000-1000-8000-00805f9b34fb" // 受信・Notify用
#define RENOGY_CHAR_RX_UUID "0000fff1-0000-1000-8000-00805f9b34fb"
#define RENOGY_SERVICE_TX_UUID "0000ffd0-0000-1000-8000-00805f9b34fb" // 送信・Write用
#define RENOGY_CHAR_TX_UUID "0000ffd1-0000-1000-8000-00805f9b34fb"

#endif

// 送信コマンド (QUERY_BATTERY_STATUS)
const uint8_t LT_BAT_QUERY_STATUS_CMD[] = {0x00, 0x00, 0x04, 0x01, 0x13, 0x55, 0xAA, 0x17};
const uint8_t LT_BAT_DISCHARGE_ON_CMD[] = {0x00, 0x00, 0x04, 0x01, 0x0C, 0x55, 0xAA, 0x10};
const uint8_t LT_BAT_DISCHARGE_OFF_CMD[] = {0x00, 0x00, 0x04, 0x01, 0x0D, 0x55, 0xAA, 0x11};

struct BleDeviceInfo
{
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
static unsigned long g_last_ble_scan_msec = 0;

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

/** >>>> AWS */
#ifdef USE_CAT_M
#include "catm.cpp.h"
#else
void connectWifi()
{
    M5.Display.fillScreen(BLACK); // 画面を黒でクリア
    M5.Display.setTextSize(1);    // 文字サイズ設定
    M5.Display.setCursor(0, 0);   // 左上にカーソルセット
    M5.Display.println("Connecting...");
    M5.Display.println("Wi-Fi...");

    WiFi.mode(WIFI_STA);
    WiFi.begin(config.WIFI_SSID, config.WIFI_PASSWORD);

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
#include "iot.cpp.h"
/** <<<< AWS */

/** >>>> BLE */
#include "ble.cpp.h"
/** BLE <<<< */

#include "network.cpp.h"

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
    USBSerial.setRxBufferSize(4096);
    USBSerial.println("DBG : setup started.");

    loadConfig();

    // aws
#ifdef USE_CAT_M
    if(config.CATM_APN == "" || config.THING_NAME == "")
    {
        //passthrough
    }
    else
    {
        SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
        connectCATM();
#else
        connectWifi();
#endif
#ifdef USE_AWS
        connectAWS();
#else
        connectTB();
    }
#endif

    // ble
    if(config.BATTERY_NAME == "" || config.CHARGER_NAME == "")
    {
        //passthrough
    }
    else
    {
        NimBLEDevice::init("");
        NimBLEScan *pBLEScan = NimBLEDevice::getScan();
        pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
        pBLEScan->setInterval(45);
        pBLEScan->setWindow(15);
        pBLEScan->setActiveScan(true);
        pBLEScan->start(10, false);
        g_last_ble_scan_msec = millis();
    }

    // xTaskCreatePinnedToCore(
    //     servoTask,  // 実行する関数
    //     "ServoTask", // タスク名
    //     4096, // スタックサイズ
    //     NULL, // パラメータ
    //     1, // 優先度
    //     NULL, // ハンドル
    //     0 // コア0で実行
    // );

    // xTaskCreatePinnedToCore(
    //     dialTask,       // 実行する関数
    //     "DialoderTask", // タスク名
    //     4096,           // スタックサイズ
    //     NULL,           // パラメータ
    //     1,              // 優先度
    //     NULL,           // ハンドル
    //     0               // コア0で実行
    // );

    xTaskCreate(
        networkTask,
        "NetworkTask",
        4096,
        nullptr,
        1,
        nullptr
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
        M5.Lcd.setTextColor(WHITE, BLACK); // 文字色
        M5.Lcd.setTextSize(1);             // 文字サイズ設定
        M5.Lcd.setTextFont(2);             // フォント
        M5.Lcd.setCursor(0, 0);            // カーソル座標指定
        LCD_printf("ID: %s\n", config.THING_NAME.c_str()); // name
        LCD_printf("DIAL: %d\n", g_dial_pos);
        LCD_printf("USB: %s\n", USBSerial ? "1" : "0");
        if (msec - g_chg_updated_msec < 5000)
        {
            LCD_printf("PWR: %s\n", g_pwr_ctl ? "ON" : "OFF");
        }
        else if (advDevice_chg.connected)
        {
            LCD_printf("PWR: WAITING...\n");
        }
        else
        {
            LCD_printf("PWR: -\n");
        }
        if (msec - g_bat_updated_msec < 5000)
        {
            LCD_printf("BAT: %d%%, %.1fC\n", g_bat_soc, g_bat_temp);
        }
        else if (advDevice_bat.connected)
        {
            LCD_printf("BAT: WAITING...\n");
        }
        else
        {
            LCD_printf("BAT: -\n");
        }

        // M5.Lcd.drawFastHLine(0, 50, 128, WHITE);           // 指定座標から横線

        // M5.Lcd.setCursor(0, 50);                           // カーソル座標指定

        if (M5.BtnA.wasPressed())
        {
            USBSerial.println("DBG : wasPressed");
            DinMeter.Encoder.readAndReset();
            // DinMeter.Encoder.write(0);
        }
        if (long_press == false && M5.BtnA.pressedFor(3000))
        {
            USBSerial.println("DBG : wasLongPressed");
            long_press = true;

            if (advDevice_bat.connected)
            {
                if (advDevice_bat.pClient->isConnected())
                {
                    if (advDevice_bat.pWriteChar != nullptr)
                    {
                        M5.Lcd.setTextColor(RED, BLACK); // 文字色

                        if (advDevice_chg.connected)
                        {
                            LCD_printf("Battery discharg off...\n");
                            delay(3000);
                            advDevice_bat.pWriteChar->writeValue(LT_BAT_DISCHARGE_OFF_CMD, sizeof(LT_BAT_DISCHARGE_OFF_CMD), true);
                            delay(1000);
                        }
                        else
                        {
                            LCD_printf("Battery discharg on...\n");
                            delay(3000);
                            advDevice_bat.pWriteChar->writeValue(LT_BAT_DISCHARGE_ON_CMD, sizeof(LT_BAT_DISCHARGE_ON_CMD), true);
                            delay(1000);
                        }
                    }
                }
            }

            M5.Display.println("Rebooting...");
            delay(3000);
            ESP.restart();
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

    iot_loop();

    ble_loop();

    //serial
    while (USBSerial && USBSerial.available() > 0)
    {
        int c = USBSerial.read();
        if (c == '\n')
        {
            _read_line.push_back('\0');
            USBSerial.printf("ECH %s\n", (char *)_read_line.data());
            if(strncmp((char *)_read_line.data(), "load_config ", 12) == 0)
            {
                bool ret = loadConfig_from_json((char *)_read_line.data() + 12);
                if(ret){
                    saveConfig();
                    //loadConfig();

                    M5.Display.println("Config loaded. Rebooting...");
                    delay(5000);   // 画面やシリアルに文字を出力し切るための少しの猶予
                    ESP.restart(); // システム再起動
                }
            }
            else if(strcmp((char *)_read_line.data(), "clear_config") == 0)
            {
                Preferences prefs;

                prefs.begin("config", false);
                prefs.clear();
                prefs.end();
                    
                M5.Display.println("Config cleared. Rebooting...");
                delay(5000);   // 画面やシリアルに文字を出力し切るための少しの猶予
                ESP.restart(); // システム再起動
            }
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