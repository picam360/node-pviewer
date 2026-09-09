extern void setPwrCtl(bool turnOn);

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
    while (!client.connect(config.THING_NAME))
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
        g_pwr_ctl_set_required = true;

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
        while (!client.connect(config.THING_NAME.c_str(), config.TB_TOKEN.c_str(), ""))
        {
            if (millis() - startAttemptTime >= TIMEOUT_MS)
            {
                USBSerial.println("\nConnection timeout! Rebooting...");
                M5.Display.fillScreen(BLACK);
                M5.Display.setCursor(0, 0);
                M5.Display.setTextColor(RED); // 成功時は緑に
                M5.Display.println("Timeout. Rebooting...");
                delay(5000);   // 画面やシリアルに文字を出力し切るための少しの猶予
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


void iot_loop()
{
    unsigned long msec = millis();

    // mqtt

    if(config.CATM_APN == "" || config.THING_NAME == "")
    {
        //passthrough;
    }
    else
    {
        client.loop();
        if (!client.connected())
        {
#ifdef USE_AWS
            connectAWS();
#else
            connectTB();
#endif
        }

        if (g_pwr_ctl_set_required)
        {
            g_pwr_ctl_set_required = false;
            setPwrCtl(g_pwr_ctl);
        }

        // 10秒ごとに送信
        static unsigned long lastMillis = 0;
        if (millis() - lastMillis > 10000)
        {
            lastMillis = millis();

            JsonDocument doc; // ArduinoJson v7の書き方
            doc["time"] = millis();
            if (msec - g_bat_updated_msec < 5000)
            {
                doc["bat_soc"] = g_bat_soc;
                doc["bat_temp"] = g_bat_temp;
            }
            else
            {
                doc["bat_soc"] = -1;
                doc["bat_temp"] = -99;
            }
            if (msec - g_chg_updated_msec < 5000 && !g_pwr_ctl_set_required)
            {
                doc["pwr_ctl"] = g_pwr_ctl ? 1 : 0;
            }
            else
            {
                doc["pwr_ctl"] = -1;
            }

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
    }
}