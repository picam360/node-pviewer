#include <Arduino.h>
#include <NimBLEDevice.h>
#include <HTTPClient.h>

#include <unordered_map>

//#define TARGET_DEVICE_ATOMS3
#define TARGET_DEVICE_M5DINMETER
#if defined(TARGET_DEVICE_ATOMS3)
#include <M5AtomS3.h> // ATOMS3用ライブラリ
#elif defined(TARGET_DEVICE_M5DINMETER)
#include <M5DinMeter.h>
#define PWR_CTR_PIN 15
#endif

// debug flgs

//params
static bool g_pwr_ctl = false;
static int g_bat_soc = 0;

// ble
// ターゲットのBLE情報
const char* TARGET_NAME = "L-12200BNN160-C00028";
const char* TARGET_ADDRESS = "C8:47:80:46:03:C0";

// UUIDの設定
static BLEUUID SERVICE_UUID((uint16_t)0xFFE0);
static BLEUUID READ_UUID((uint16_t)0xFFE1);
static BLEUUID WRITE_UUID((uint16_t)0xFFE2);

// 送信コマンド (QUERY_BATTERY_STATUS)
const uint8_t QUERY_CMD[] = {0x00, 0x00, 0x04, 0x01, 0x13, 0x55, 0xAA, 0x17};

static NimBLEClient* pClient = nullptr;
static NimBLERemoteCharacteristic* pWriteChar = nullptr;
static bool doConnect = false;
static bool connected = false;
static NimBLEAdvertisedDevice* advDevice;

static std::vector<uint8_t> _read_line;
static std::string _ssid = "ERROR_NO_RESPONSE";
static std::string _ip_address = "ERROR_NO_RESPONSE";

//system
static int status_loop_count = 0;
static unsigned long last_status_msec = 0;
static long status_interval_msec = 100;

//#define DBG_OUT_ENABLE
#define DBGP_BUFF_SIZE 512
static char g_last_dbgp_msg[DBGP_BUFF_SIZE];
void dbgPrintf(char *format, ...) {
#ifdef DBG_OUT_ENABLE
  if(USBSerial){
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

/** >>>> BLE */
// エンディアン変換用（Pythonのrev_hexをシミュレート）
uint32_t get_uint32_le(const uint8_t* data, int start) {
    return (data[start+3] << 24) | (data[start+2] << 16) | (data[start+1] << 8) | data[start];
}

uint16_t get_uint16_le(const uint8_t* data, int start) {
    return (data[start+1] << 8) | data[start];
}

// データパースと画面表示
void parse_litime(const uint8_t* data, size_t length) {
    if (length < 90) return; // 最低限必要なデータ長をチェック

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
    
    if(USBSerial){
        USBSerial.printf("SOC: %d%%, V: %.2fV, A: %.2fA, Temp: %.1fC\n", soc, total_voltage, current, cell_temp);
    }
}

// 通知（Notify）コールバック
static void notifyCallback(NimBLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    parse_litime(pData, length);
}

// BLEスキャンコールバック
class MyAdvertisedDeviceCallbacks: public NimBLEAdvertisedDeviceCallbacks {
    void onResult(NimBLEAdvertisedDevice* advertisedDevice) {
        if (advertisedDevice->getAddress().toString() == TARGET_ADDRESS || advertisedDevice->getName() == TARGET_NAME) {
            NimBLEDevice::getScan()->stop();
            advDevice = advertisedDevice;
            doConnect = true;
        }
    }
};

// 接続処理
bool connectToServer() {
    if (pClient == nullptr) {
        pClient = NimBLEDevice::createClient();
    }
    
    if (!pClient->connect(advDevice)) return false;
    
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

    NimBLERemoteService* pRemoteService = pClient->getService(SERVICE_UUID);
    
    if (pRemoteService == nullptr) { 
        if(USBSerial){
            USBSerial.println("[エラー] サービス(0xFFE0)が見つかりませんでした。");
        }
        pClient->disconnect(); 
        return false; 
    }
    if(USBSerial){
        USBSerial.println("[BLE] サービス(0xFFE0)の特定に成功！");
    }

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

    NimBLERemoteCharacteristic* pReadChar = pRemoteService->getCharacteristic(READ_UUID);
    if (pReadChar && pReadChar->canNotify()) {
        pReadChar->subscribe(true, notifyCallback);
        if(USBSerial){
            USBSerial.println("[BLE] Notify（通知）の登録完了！");
        }
    } else {
        if(USBSerial){
            USBSerial.println("[エラー] READキャラスティックが見つからない、またはNotifyに対応していません。");
        }
        pClient->disconnect();
        return false;
    }

    pWriteChar = pRemoteService->getCharacteristic(WRITE_UUID);
    if (pWriteChar == nullptr) {
        if(USBSerial){
            USBSerial.println("[エラー] WRITEキャラスティックが見つかりません。");
        }
        pClient->disconnect();
        return false;
    }

    if(USBSerial){
        USBSerial.println("[BLE] すべての接続・初期化が正常に完了しました！");
    }
    return true;
}

/** BLE <<<< */

void servoTask(void *pvParameters) {
}
static int32_t g_dial_pos = 0;
void dialTask(void *pvParameters) {
    while (1) {
        int32_t dial_pos = DinMeter.Encoder.read();
        if (dial_pos != g_dial_pos) {
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

    USBSerial.begin(115200);//need to be called for USBSerial.isPlugged=true
    //USBSerial.setRxBufferSize(4096);//for big rtcm data
    if(USBSerial){
        USBSerial.println("DBG : setup started.");
    }

    // ble
    NimBLEDevice::init("");
    NimBLEScan* pBLEScan = NimBLEDevice::getScan();
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
        dialTask,  // 実行する関数
        "DialoderTask", // タスク名
        4096, // スタックサイズ
        NULL, // パラメータ
        1, // 優先度
        NULL, // ハンドル
        0 // コア0で実行
    );
}

void LCD_printf(const char *format, ...) {
    char buff[32];
    va_list args;
    va_start(args, format);
    vsnprintf(buff, 32, format, args);
    va_end(args);

    int x = M5.Lcd.textWidth(buff);
    int y = M5.Lcd.getCursorY();
    int w = 128 - x;
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

        if (M5.BtnA.wasPressed()) {
            button_state = true;
        } else if (M5.BtnA.wasReleased()) {
            if(button_state){
                display_mode++;
                
                M5.Lcd.fillScreen(BLACK); // 背景
            }
            button_state = false;
        }

        M5.Lcd.setTextColor(WHITE, BLACK);                 // 文字色
        M5.Lcd.setTextFont(2);                             // フォント
        M5.Lcd.setCursor(0, 0);                            // カーソル座標指定
        LCD_printf("SSID:%.11s\n", _ssid.c_str());         // アクセスポイント時のSSID表示
        M5.Lcd.setTextColor(ORANGE, BLACK);                // 文字色
        LCD_printf("IP:%.13s\n", _ip_address.c_str());     // IPアドレス表示
        M5.Lcd.drawFastHLine(0, 34, 128, WHITE);           // 指定座標から横線

        M5.Lcd.setCursor(0, 38);                           // カーソル座標指定
        M5.Lcd.setTextColor(CYAN, BLACK);                  // 文字色
        switch(display_mode%3){
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

        //DinMeter.Speaker.tone(8000, 20);
        M5.Lcd.setTextColor(WHITE, BLACK);                 // 文字色
        M5.Lcd.setTextFont(2);                             // フォント
        M5.Lcd.setCursor(0, 0);                            // カーソル座標指定
        LCD_printf("DIAL: %d\n", g_dial_pos);         // アクセスポイント時のSSID表示
        LCD_printf("USB: %s\n", USBSerial ? "1" : "0"); 
        LCD_printf("PWR: %s\n", g_pwr_ctl ? "ON" : "OFF");
        if(connected){
            LCD_printf("BAT: %d%%\n", g_bat_soc);
        }else{
            LCD_printf("BAT: -\n");
        }

        M5.Lcd.drawFastHLine(0, 50, 128, WHITE);           // 指定座標から横線
        
        //M5.Lcd.setCursor(0, 50);                           // カーソル座標指定

        if (M5.BtnA.wasPressed()) {
            DinMeter.Encoder.readAndReset();
            //DinMeter.Encoder.write(0);
            if(USBSerial){
                USBSerial.println("DBG : wasPressed");
            }
        }
        if (long_press == false && M5.BtnA.pressedFor(3000)) {
            long_press = true;

            g_pwr_ctl = !g_pwr_ctl;
            digitalWrite(PWR_CTR_PIN, g_pwr_ctl ? HIGH : LOW);

            if(USBSerial){
                USBSerial.println("DBG : wasLongPressed");
            }
        }
        if (M5.BtnA.wasReleased()) {
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

    //ble
    if (doConnect) {
        doConnect = false;
        if (connectToServer()) {
            connected = true;
        } else {
            connected = false;
            delay(2000);
            NimBLEDevice::getScan()->start(10, false); // 再スキャン
        }
    }

    // 接続中なら1秒ごとにリクエストコマンドを送信
    if (connected) {
        if (pClient->isConnected()) {
            if (pWriteChar != nullptr) {
                pWriteChar->writeValue(QUERY_CMD, sizeof(QUERY_CMD), true);
            }
            delay(1000);
        } else {
            connected = false;
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