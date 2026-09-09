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

struct NetworkCheckResult {
    uint32_t timestamp;

    bool validIp;

    int sent;
    int received;
    int lost;

    float packetLoss;

    uint32_t minRtt;
    float avgRtt;
    uint32_t maxRtt;
} network_check_result;
const uint32_t CHECK_NETWORK_CYCLE = 10000;

NetworkCheckResult checkNetwork(const String& ipString)
{
    NetworkCheckResult result = {};
    result.timestamp = millis();

    IPAddress ip;
    if (!ip.fromString(ipString)) {
        USBSerial.printf(
            "Invalid PING_IP: %s\n",
            ipString.c_str()
        );

        result.validIp = false;
        return result;
    }

    result.validIp = true;

    const int count = 20;

    uint32_t totalRtt = 0;
    result.minRtt = UINT32_MAX;
    result.maxRtt = 0;

    USBSerial.printf(
        "PING %s: %d packets\n",
        ipString.c_str(),
        count
    );

    for (int i = 0; i < count; i++) {

        const bool success = Ping.ping(ip, 1);

        result.sent++;

        if (success) {
            const uint32_t rtt =
                (uint32_t)Ping.averageTime();

            result.received++;
            totalRtt += rtt;

            if (rtt < result.minRtt) {
                result.minRtt = rtt;
            }

            if (rtt > result.maxRtt) {
                result.maxRtt = rtt;
            }

            USBSerial.printf(
                "  [%02d/%02d] Reply: %lu ms\n",
                i + 1,
                count,
                rtt
            );
        }
        else {
            result.lost++;

            USBSerial.printf(
                "  [%02d/%02d] Timeout\n",
                i + 1,
                count
            );
        }

        delay(100);
    }

    result.packetLoss =
        ((float)result.lost / (float)result.sent) * 100.0f;

    result.avgRtt =
        result.received > 0
            ? (float)totalRtt / (float)result.received
            : 0.0f;

    if (result.received == 0) {
        result.minRtt = 0;
    }

    USBSerial.printf("\n");
    USBSerial.printf("===== PING RESULT =====\n");
    USBSerial.printf("Target      : %s\n", ipString.c_str());
    USBSerial.printf("Sent        : %d\n", result.sent);
    USBSerial.printf("Received    : %d\n", result.received);
    USBSerial.printf("Lost        : %d\n", result.lost);
    USBSerial.printf("Packet Loss : %.1f %%\n", result.packetLoss);

    if (result.received > 0) {
        USBSerial.printf(
            "Min RTT     : %lu ms\n",
            result.minRtt
        );
        USBSerial.printf(
            "Avg RTT     : %.1f ms\n",
            result.avgRtt
        );
        USBSerial.printf(
            "Max RTT     : %lu ms\n",
            result.maxRtt
        );
    }
    else {
        USBSerial.printf("RTT         : N/A\n");
    }

    USBSerial.printf("=======================\n");

    return result;
}

void networkTask(void *parameter)
{
    while (true) {

        if (WiFi.status() == WL_CONNECTED && config.PING_IP != "")
        {
            network_check_result = checkNetwork(config.PING_IP);
        }
        else
        {
            USBSerial.println("WiFi disconnected - skip ping");
        }

        vTaskDelay(pdMS_TO_TICKS(CHECK_NETWORK_CYCLE));
    }
}