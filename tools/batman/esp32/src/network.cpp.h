#include <ESP32Ping.h>

void connectWifi()
{
    M5.Display.fillScreen(BLACK); // 画面を黒でクリア
    M5.Display.setTextSize(1);    // 文字サイズ設定
    M5.Display.setCursor(0, 0);   // 左上にカーソルセット
    M5.Display.println("Connecting...");
    M5.Display.println("Wi-Fi...");

    WiFi.mode(WIFI_STA);
    WiFi.begin(config.WIFI_SSID, config.WIFI_PASSWORD);

    for (int i=0;i<20;i++)
    {
        if(WiFi.status() == WL_CONNECTED){
            break;
        }
        delay(500);
        M5.Display.print("."); // 画面にドットを追加していく
        USBSerial.print(".");  // シリアルにも出力
    }

    // 接続完了の表示
    if(WiFi.status() != WL_CONNECTED){
        M5.Display.println("\nFAIL!");
    }else{
        M5.Display.println("\OK!");
    }
    delay(2000);                  // メッセージを確認するために少し待機
    M5.Display.fillScreen(BLACK); // 画面をクリアしてメイン処理へ
}

void checkNetwork(String ipString)
{
    IPAddress ip;
    if (!ip.fromString(ipString)) {
        USBSerial.printf("Invalid PING_IP: %s\n", ipString.c_str());
        return;
    }

    const int count = 20;

    int successCount = 0;
    int failedCount = 0;

    uint32_t totalRtt = 0;
    uint32_t minRtt = UINT32_MAX;
    uint32_t maxRtt = 0;

    USBSerial.printf(
        "PING %s: %d packets\n",
        ipString.c_str(),
        count
    );

    for (int i = 0; i < count; i++) {

        // 1パケットだけ送信
        const bool success = Ping.ping(ip, 1);

        if (success) {
            const uint32_t rtt = (uint32_t)Ping.averageTime();

            successCount++;
            totalRtt += rtt;

            if (rtt < minRtt) {
                minRtt = rtt;
            }

            if (rtt > maxRtt) {
                maxRtt = rtt;
            }

            USBSerial.printf(
                "  [%02d/%02d] Reply: %lu ms\n",
                i + 1,
                count,
                rtt
            );
        }
        else {
            failedCount++;

            USBSerial.printf(
                "  [%02d/%02d] Timeout\n",
                i + 1,
                count
            );
        }

        delay(100);
    }

    // Packet Loss
    const float packetLoss =
        ((float)failedCount / (float)count) * 100.0f;

    // 成功したパケットだけで平均RTTを計算
    const float avgRtt =
        successCount > 0
            ? (float)totalRtt / (float)successCount
            : 0.0f;

    USBSerial.printf("\n");
    USBSerial.printf("===== PING RESULT =====\n");
    USBSerial.printf("Target      : %s\n", ipString.c_str());
    USBSerial.printf("Sent        : %d\n", count);
    USBSerial.printf("Received    : %d\n", successCount);
    USBSerial.printf("Lost        : %d\n", failedCount);
    USBSerial.printf("Packet Loss : %.1f %%\n", packetLoss);

    if (successCount > 0) {
        USBSerial.printf("Min RTT     : %lu ms\n", minRtt);
        USBSerial.printf("Avg RTT     : %.1f ms\n", avgRtt);
        USBSerial.printf("Max RTT     : %lu ms\n", maxRtt);
    }
    else {
        USBSerial.printf("RTT         : N/A\n");
    }

    USBSerial.printf("=======================\n");
}

void networkTask(void *parameter)
{
    while (true) {

        if (WiFi.status() == WL_CONNECTED && config.PING_IP != "")
        {
            checkNetwork(config.PING_IP);
        }
        else
        {
            USBSerial.println("WiFi disconnected - skip ping");
        }

        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}