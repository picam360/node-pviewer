
// Modbus RTU CRC-16 計算関数
uint16_t calculateModbusCRC(const uint8_t *data, uint8_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 1)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}
uint8_t calcLiTimeChecksum(const uint8_t *data, size_t length)
{

    // how to use
    //  uint8_t cmd[8] = {0x00, 0x00, 0x04, 0x01, 0x0d, 0x55, 0xAA, 0x00};
    //  cmd[7] = calcLiTimeChecksum(cmd, 7);
    //  advDevice_bat.pWriteChar->writeValue(cmd, sizeof(cmd), true);

    uint16_t sum = 0;
    for (size_t i = 0; i < length; i++)
    {
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
    if (length <= 90)
        return; // 最低限必要なデータ長をチェック

    // 電圧・電流・容量の解析
    float total_voltage = get_uint32_le(data, 8) / 1000.0;

    int16_t raw_current = get_uint16_le(data, 48);
    // Pythonの「r = ~raw_current; (-r if r > 0 else raw_current)」に相当する符号付き処理
    float current = ((int16_t)raw_current) / 1000.0;

    int16_t cell_temp_raw = get_uint16_le(data, 52);
    float cell_temp = (float)cell_temp_raw; // 2の補数処理はint16_tのキャストで自動適用されます

    uint16_t battery_state_code = get_uint16_le(data, 88);

    int soc = data[90]; // 90番目のバイト

    g_bat_soc = soc;
    g_bat_temp = cell_temp;
    g_bat_updated_msec = millis();

    USBSerial.printf("SOC: %d%%, V: %.2fV, A: %.2fA, Temp: %.1fC\n", soc, total_voltage, current, cell_temp);
}

// 負荷をON/OFFする関数 (引数に true を渡すとON、false でOFF)
void setPwrCtl(bool turnOn)
{

    // digitalWrite(PWR_CTR_PIN, turnOn ? HIGH : LOW);

    if (advDevice_chg.pWriteChar == nullptr)
    {
        USBSerial.println("[エラー] TXキャラスティックが準備されていません");
        return;
    }

    // [ID] [Func=0x06] [Reg_H] [Reg_L] [Data_H] [Data_L] [CRC_L] [CRC_H]
    uint8_t cmdOn[] = {0xFF, 0x06, 0x01, 0x0A, 0x00, 0x01, 0x69, 0xF4};
    uint8_t cmdOff[] = {0xFF, 0x06, 0x01, 0x0A, 0x00, 0x00, 0xA8, 0x34};

    // 送信するコマンドを選択
    uint8_t *cmd = turnOn ? cmdOn : cmdOff;
    size_t cmdSize = turnOn ? sizeof(cmdOn) : sizeof(cmdOff);

    uint16_t crc = calculateModbusCRC(cmd, 6);
    // ModbusのCRCは リトルエンディアン (下位バイトが先)
    cmd[6] = crc & 0xFF;
    cmd[7] = (crc >> 8) & 0xFF;

    // 通信方式を自動判定して送信
    if (advDevice_chg.pWriteChar->canWriteNoResponse())
    {
        advDevice_chg.pWriteChar->writeValue(cmd, cmdSize, false);
    }
    else
    {
        advDevice_chg.pWriteChar->writeValue(cmd, cmdSize, true);
    }
    delay(1000);

    if (turnOn)
    {
        USBSerial.println("[CHG] 負荷を【ON】にするコマンドを送信しました");
    }
    else
    {
        USBSerial.println("[CHG] 負荷を【OFF】にするコマンドを送信しました");
    }
}

// 通知（Notify）コールバック
static void notifyCallback_bat(NimBLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify)
{
    parse_litime(pData, length);
}

// Renogyからの応答を受け取るコールバック
void notifyCallback_chg(NimBLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify)
{
    // もし 7バイトあり、ID=0x01、Read応答(0x03)、データ長2バイト(0x02) なら
    if (length >= 7 /*&& pData[0] == 0x01*/ && pData[1] == 0x03 && pData[2] == 0x02)
    {

        // pData[3] が上位バイト、pData[4] が下位バイト
        uint16_t value = (pData[3] << 8) | pData[4];
        USBSerial.printf("[CHG:read 1 word] 0x%04X\n", value);

        // 0x1020
        //  負荷のON/OFFは「下位バイト(pData[4]) の ビット15」に格納されている
        uint8_t loadState = (value >> 15) & 0x01;

        if (g_pwr_ctl_set_required)
        {
            // ignore
        }
        else
        {
            if (loadState == 1)
            {
                g_pwr_ctl = true;
            }
            else if (loadState == 0)
            {
                g_pwr_ctl = false;
            }
            g_chg_updated_msec = millis();
        }
    }
    // 書き込み(0x06)に対するエコーバック応答の場合
    else if (length >= 8 /*&& pData[0] == 0x01*/ && pData[1] == 0x06)
    {
        USBSerial.println("[CHG] 書き込み(ON/OFF)コマンドが正常に受理されました");
    }
    else
    {
        // エラー等のその他の応答
        USBSerial.print("[CHG] 別の応答を受信: ");
        for (size_t i = 0; i < length; i++)
        {
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
        USBSerial.println("[BAT:エラー] サービス(0xFFE0)が見つかりませんでした。");
        advDevice_bat.pClient->disconnect();
        return false;
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

    NimBLERemoteCharacteristic *pReadChar = pRemoteService->getCharacteristic(LT_BAT_READ_UUID);
    if (pReadChar && pReadChar->canNotify())
    {
        pReadChar->subscribe(true, notifyCallback_bat);
        USBSerial.println("[BAT] Notify（通知）の登録完了！");
    }
    else
    {
        USBSerial.println("[BAT:エラー] READキャラスティックが見つからない、またはNotifyに対応していません。");
        advDevice_bat.pClient->disconnect();
        return false;
    }

    advDevice_bat.pWriteChar = pRemoteService->getCharacteristic(LT_BAT_WRITE_UUID);
    if (advDevice_bat.pWriteChar == nullptr)
    {
        USBSerial.println("[BAT:エラー] WRITEキャラスティックが見つかりません。");
        advDevice_bat.pClient->disconnect();
        return false;
    }

    USBSerial.println("[BAT] すべての接続・初期化が正常に完了しました！");
    return true;
}
bool connectToBle_chg()
{
    if (advDevice_chg.pClient == nullptr)
    {
        advDevice_chg.pClient = NimBLEDevice::createClient();
    }

    if (!advDevice_chg.pClient->connect(advDevice_chg.addr))
    {
        USBSerial.println("[エラー] チャージャーへの接続に失敗しました。");
        return false;
    }

#ifdef LT_CHG
// 2. サービスの取得
    NimBLERemoteService *pService = advDevice_chg.pClient->getService(LT_CHG_SERVICE_UUID);
    if (pService == nullptr)
    {
        USBSerial.println("[エラー] LiTimeサービスが見つかりませんでした。");
        advDevice_chg.pClient->disconnect();
        return false;
    }

    // 3. RX (受信・Notify) キャラスティックの設定
    NimBLERemoteCharacteristic *pNotifyChar = pService->getCharacteristic(LT_CHG_READ_UUID);
    if (pNotifyChar && pNotifyChar->canNotify())
    {
        // 第二引数は通知を受け取るコールバック関数
        if (!pNotifyChar->subscribe(true, notifyCallback_chg))
        {
            USBSerial.println("[CHG:エラー] Notifyの購読(subscribe)に失敗しました。");
            advDevice_chg.pClient->disconnect();
            return false;
        }
        USBSerial.println("[CHG] Notify（通知）の登録完了！");
    }
    else
    {
        USBSerial.println("[CHG:エラー] RX(Notify)キャラスティックが見つからないか、Notify非対応です。");
        advDevice_chg.pClient->disconnect();
        return false;
    }

    // 4. TX (送信・Write) キャラスティックの取得
    advDevice_chg.pWriteChar = pService->getCharacteristic(LT_CHG_WRITE_UUID);
    if (advDevice_chg.pWriteChar == nullptr || (!advDevice_chg.pWriteChar->canWrite() && !advDevice_chg.pWriteChar->canWriteNoResponse()))
    {
        USBSerial.println("[CHG:エラー] TX(Write)キャラスティックが見つからないか、書き込み不可です。");
        advDevice_chg.pClient->disconnect();
        return false;
    }

#else

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
#endif

    USBSerial.println("[CHG] すべての接続・初期化が正常に完了しました！");
    return true;
}

void ble_loop()
{
    unsigned long msec = millis();
    //finding ble
    if(config.BATTERY_NAME == "" || config.CHARGER_NAME == "")
    {
        // passthrough
    }
    else
    {
        BleDeviceInfo devInfo = {};
        if (xQueueReceive(bleQueue, &devInfo, 0))
        {
            USBSerial.printf("BLE dev found : %s(%s)\n", devInfo.name, devInfo.addr.toString().c_str());
            if (advDevice_bat.name[0] == '\0' && strcmp(devInfo.name, config.BATTERY_NAME.c_str()) == 0)
            {
                advDevice_bat = devInfo;
                advDevice_bat.doConnect = true;
                USBSerial.printf("BAT dev found! : %s(%s)\n", devInfo.name, devInfo.addr.toString().c_str());
            }
            if (advDevice_chg.name[0] == '\0' && strcmp(devInfo.name, config.CHARGER_NAME.c_str()) == 0)
            {
                advDevice_chg = devInfo;
                advDevice_chg.doConnect = true;
                USBSerial.printf("CHG dev found! : %s(%s)\n", devInfo.name, devInfo.addr.toString().c_str());
            }
            if (advDevice_bat.name[0] != '\0' && advDevice_chg.name[0] != '\0')
            {
                NimBLEDevice::getScan()->stop();

                USBSerial.println("BLE: All devices have been found!");
            }
        }
    }
    // ble
    {
        if(config.BATTERY_NAME == "")
        {
            //passthough
        }
        else if (!advDevice_bat.connected && advDevice_bat.doConnect)
        {
            advDevice_bat.doConnect = false;
            if (connectToBle_bat())
            {
                advDevice_bat.connected = true;
            }
            else
            {
                //danger?//memset(&advDevice_bat, 0, sizeof(advDevice_bat));
                advDevice_bat.name[0] = '\0';
                advDevice_bat.pClient = nullptr;
                advDevice_bat.pWriteChar = nullptr;
                advDevice_bat.doConnect = false;
                advDevice_bat.connected = false;
                advDevice_bat.addr = NimBLEAddress();

                delay(2000);
                NimBLEDevice::getScan()->start(10, false); // 再スキャン
                g_last_ble_scan_msec = millis();
            }
        }
        if(config.CHARGER_NAME == "")
        {
            //passthough
        }
        else if (!advDevice_chg.connected && advDevice_chg.doConnect)
        {
            advDevice_chg.doConnect = false;
            if (connectToBle_chg())
            {
                advDevice_chg.connected = true;
            }
            else
            {
                //danger?//memset(&advDevice_chg, 0, sizeof(advDevice_chg));
                advDevice_chg.name[0] = '\0';
                advDevice_chg.pClient = nullptr;
                advDevice_chg.pWriteChar = nullptr;
                advDevice_chg.doConnect = false;
                advDevice_chg.connected = false;
                advDevice_chg.addr = NimBLEAddress();

                delay(2000);
                NimBLEDevice::getScan()->start(10, false); // 再スキャン
                g_last_ble_scan_msec = millis();
            }
        }
        if(config.BATTERY_NAME == "" || config.CHARGER_NAME == "")
        {
            // passthrough
        }
        else if (advDevice_bat.name[0] != '\0' && advDevice_chg.name[0] != '\0')
        {
            // passthrough
        }
        else if (msec - g_last_ble_scan_msec > 30 * 1000)//timeout
        {
            NimBLEDevice::getScan()->stop();
            delay(1000);
            NimBLEDevice::getScan()->start(10, false); // 再スキャン
            g_last_ble_scan_msec = millis();

            USBSerial.println("BLE: Timeout, Rescan");
        }

        // 接続中なら1秒ごとにリクエストコマンドを送信
        if (advDevice_bat.connected)
        {
            if (advDevice_bat.pClient->isConnected())
            {
                if (advDevice_bat.pWriteChar != nullptr)
                {
                    advDevice_bat.pWriteChar->writeValue(LT_BAT_QUERY_STATUS_CMD, sizeof(LT_BAT_QUERY_STATUS_CMD), true);
                }
                delay(1000);
            }
            else
            {
                memset(&advDevice_bat, 0, sizeof(advDevice_bat));
                delay(2000);
                NimBLEDevice::getScan()->start(10, false);
                g_last_ble_scan_msec = millis();
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
                    uint8_t cmd[] = {0xFF, 0x03, 0x01, 0x20, 0x00, 0x01, 0x00, 0x00};

                    uint16_t crc = calculateModbusCRC(cmd, 6);
                    // ModbusのCRCは リトルエンディアン (下位バイトが先)
                    cmd[6] = crc & 0xFF;
                    cmd[7] = (crc >> 8) & 0xFF;

                    advDevice_chg.pWriteChar->writeValue(cmd, sizeof(cmd), false);
                }
                delay(1000);
            }
            else
            {
                memset(&advDevice_chg, 0, sizeof(advDevice_chg));
                delay(2000);
                NimBLEDevice::getScan()->start(10, false);
                g_last_ble_scan_msec = millis();
            }
        }
    }
}