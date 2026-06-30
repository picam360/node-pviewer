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

// ble
#define BLE_DEVICE_NAME "chariot"
#define BLE_SRV_CHARIOT "70333680-7067-0000-0001-000000000001"
#define BLE_SRV_CHARIOT_C_RX "70333681-7067-0000-0001-000000000001"
#define BLE_SRV_CHARIOT_C_TX "70333682-7067-0000-0001-000000000001"
static NimBLEServer *_ble_svr = nullptr;
static NimBLEAdvertising *_ble_adv = nullptr;
static NimBLEService *_ble_svc = nullptr;
static NimBLECharacteristic *_ble_c_rx = nullptr;
static NimBLECharacteristic *_ble_c_tx = nullptr;
static SemaphoreHandle_t _sem_var_access;

static std::vector<uint8_t> _read_line;
static std::string _ssid = "ERROR_NO_RESPONSE";
static std::string _ip_address = "ERROR_NO_RESPONSE";

//system
static int status_loop_count = 0;
static unsigned long last_status_msec = 0;
static long status_interval_msec = 100;

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

void startAdvertising()
{
    if (_ble_adv != nullptr)
    {
        _ble_adv->stop();
    }
    _ble_adv = NimBLEDevice::getAdvertising();

    // ★ 追加：バッテリーに優しいアドバタイジング設定（スキャンレスポンスをオフにする）
    // これにより、電波を出す回数が減り、瞬発的な負荷が下がります
    _ble_adv->setScanResponse(false);

    // only one uuid is valid
    _ble_adv->addServiceUUID(_ble_svc->getUUID());
    // rtk_ble_add_service_uuid(_ble_adv);

    _ble_adv->start();
}

class BleSvrCb : public NimBLEServerCallbacks
{
    void onConnect(NimBLEServer *_ble_svr)
    {
        dbgPrintf("ble client connected\n");
        startAdvertising();
    }
    void onDisconnect(NimBLEServer *_ble_svr)
    {
        dbgPrintf("ble client disconnected\n");
    }
};

static void write_tx(std::string str)
{
    if (_ble_c_tx->getSubscribedCount() > 0)
    {
        if (xSemaphoreTake(_sem_var_access, 50) != pdFALSE)
        {
            _ble_c_tx->setValue(str);
            _ble_c_tx->notify();

            if(USBSerial){
                USBSerial.printf("DBG : BLE_TX %s\n", str.c_str());
            }

            xSemaphoreGive(_sem_var_access);
        }
    }
}

class BleChRx : public NimBLECharacteristicCallbacks
{
    void onWrite(NimBLECharacteristic *pCharacteristic)
    {
        NimBLEAttValue rxData = pCharacteristic->getValue();
        const char *cmd = rxData.c_str();
        if (strncmp(cmd, "REQ GET_SERVO_ENC", 17) == 0)
        {
        }
        else if (strncmp(cmd, "REQ FORWARD", 11) == 0)
        {
        }
        else if (strncmp(cmd, "REQ BACKWARD", 12) == 0)
        {
        }
        else
        { // relay
            if(USBSerial){
                USBSerial.printf("EXTRX %s\n", cmd);
            }
        }
    }
};
class BleChTx : public NimBLECharacteristicCallbacks
{
};

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

    if(USBSerial){
        USBSerial.begin(115200);
        USBSerial.setRxBufferSize(4096);//for big rtcm data

        USBSerial.printf("BLE Device Name : %s\n", BLE_DEVICE_NAME);
    }

    // semaphore
    _sem_var_access = xSemaphoreCreateMutex();

    NimBLEDevice::init(BLE_DEVICE_NAME);
    NimBLEDevice::setPower(ESP_PWR_LVL_N24); 
    dbgPrintf("ble addr: %s\n",
              NimBLEDevice::getAddress().toString().c_str());
    _ble_svr = NimBLEDevice::createServer();
    _ble_svr->setCallbacks(new BleSvrCb());

    _ble_svc = _ble_svr->createService(BLE_SRV_CHARIOT);
    {
        _ble_c_rx = _ble_svc->createCharacteristic(BLE_SRV_CHARIOT_C_RX, NIMBLE_PROPERTY::WRITE);
        _ble_c_rx->setCallbacks(new BleChRx());
        _ble_c_tx = _ble_svc->createCharacteristic(BLE_SRV_CHARIOT_C_TX, NIMBLE_PROPERTY::NOTIFY);
        _ble_c_tx->setCallbacks(new BleChTx());
    }
    _ble_svc->start();

    startAdvertising();

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
  
    M5.Lcd.printf(buff);
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
        LCD_printf("DIAL:%d\n", g_dial_pos);         // アクセスポイント時のSSID表示
        LCD_printf("PWR:%s\n", g_pwr_ctl ? "ON" : "OFF");         // アクセスポイント時のSSID表示

        M5.Lcd.drawFastHLine(0, 50, 128, WHITE);           // 指定座標から横線
        
        //M5.Lcd.setCursor(0, 50);                           // カーソル座標指定

        if (M5.BtnA.wasPressed()) {
            DinMeter.Encoder.readAndReset();
            //DinMeter.Encoder.write(0);
        }
        if (long_press == false && M5.BtnA.pressedFor(3000)) {
            long_press = true;

            g_pwr_ctl = !g_pwr_ctl;
            digitalWrite(PWR_CTR_PIN, g_pwr_ctl ? HIGH : LOW);
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