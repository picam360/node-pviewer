#include "ping/ping_sock.h"
#include "lwip/ip_addr.h"

void connectWifi()
{
    M5.Display.fillScreen(BLACK); // 画面を黒でクリア
    M5.Display.setTextSize(1);    // 文字サイズ設定
    M5.Display.setCursor(0, 0);   // 左上にカーソルセット
    M5.Display.println("Connecting...");
    M5.Display.println("Wi-Fi...");

    WiFi.mode(WIFI_STA);

    if (config.WIFI_STATIC_IP.length() > 0) {
        IPAddress localIP;
        IPAddress subnet;
        IPAddress gateway;

        localIP.fromString(config.WIFI_STATIC_IP);

        if (config.WIFI_SUBNET.length() > 0) {
            subnet.fromString(config.WIFI_SUBNET);
        } else {
            subnet = IPAddress(255, 255, 255, 0);
        }

        if (config.WIFI_GATEWAY.length() > 0) {
            gateway.fromString(config.WIFI_GATEWAY);
        } else {
            gateway = IPAddress(0, 0, 0, 0);
        }

        M5.Display.println("Static IP : " + config.WIFI_STATIC_IP);
        WiFi.config(localIP, gateway, subnet);
    }
    else
    {
        M5.Display.println("DHCP...");
    }
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
    volatile bool finished;
    int totalCount;
} network_check_result;
const uint32_t CHECK_NETWORK_CYCLE = 10000;

NetworkCheckResult checkNetwork(const String& ipString)
{
    NetworkCheckResult result = {};

    result.timestamp = millis();
    result.validIp = false;
    result.sent = 0;
    result.received = 0;
    result.lost = 0;
    result.packetLoss = 0.0f;
    result.minRtt = UINT32_MAX;
    result.avgRtt = 0.0f;
    result.maxRtt = 0;
    result.finished = false;
    result.totalCount = 0;

    IPAddress ip;

    if (!ip.fromString(ipString)) {
        USBSerial.printf(
            "Invalid PING_IP: %s\n",
            ipString.c_str()
        );
        return result;
    }

    result.validIp = true;

    const uint32_t count = 20;
    result.totalCount = count;

    USBSerial.printf(
        "PING %s: %lu packets\n",
        ipString.c_str(),
        count
    );

    // IPAddress -> ip_addr_t
    ip_addr_t target_addr;

    ip_addr_set_zero(&target_addr);

    IP4_ADDR(
        ip_2_ip4(&target_addr),
        ip[0],
        ip[1],
        ip[2],
        ip[3]
    );

    IP_SET_TYPE_VAL(
        target_addr,
        IPADDR_TYPE_V4
    );

    // Ping configuration
    esp_ping_config_t config = ESP_PING_DEFAULT_CONFIG();

    config.target_addr = target_addr;
    config.count = count;
    config.interval_ms = 100;
    config.timeout_ms = 300;

    // Callbacks
    esp_ping_callbacks_t callbacks = {};

    callbacks.cb_args = &result;

    // Success
    callbacks.on_ping_success =
        [](esp_ping_handle_t hdl, void *args)
    {
        NetworkCheckResult *result =
            static_cast<NetworkCheckResult *>(args);

        uint16_t seqno = 0;
        uint32_t rtt = 0;

        esp_ping_get_profile(
            hdl,
            ESP_PING_PROF_SEQNO,
            &seqno,
            sizeof(seqno)
        );

        esp_ping_get_profile(
            hdl,
            ESP_PING_PROF_TIMEGAP,
            &rtt,
            sizeof(rtt)
        );

        result->received++;

        if (rtt < result->minRtt) {
            result->minRtt = rtt;
        }

        if (rtt > result->maxRtt) {
            result->maxRtt = rtt;
        }

        result->avgRtt =
            (
                result->avgRtt *
                (result->received - 1)
                + rtt
            )
            / result->received;

        USBSerial.printf(
            "  [%02u/%02d] Reply: %lu ms\n",
            seqno,
            result->totalCount,
            rtt
        );
    };

    // Timeout
    callbacks.on_ping_timeout =
        [](esp_ping_handle_t hdl, void *args)
    {
        NetworkCheckResult *result =
            static_cast<NetworkCheckResult *>(args);

        uint16_t seqno = 0;

        esp_ping_get_profile(
            hdl,
            ESP_PING_PROF_SEQNO,
            &seqno,
            sizeof(seqno)
        );

        result->lost++;

        USBSerial.printf(
            "  [%02u/%02d] Timeout\n",
            seqno,
            result->totalCount
        );
    };

    // End
    callbacks.on_ping_end =
        [](esp_ping_handle_t hdl, void *args)
    {
        NetworkCheckResult *result =
            static_cast<NetworkCheckResult *>(args);

        result->finished = true;
    };

    // Create session
    esp_ping_handle_t ping = nullptr;

    esp_err_t err =
        esp_ping_new_session(
            &config,
            &callbacks,
            &ping
        );

    if (err != ESP_OK) {
        USBSerial.printf(
            "Failed to create ping session: %s\n",
            esp_err_to_name(err)
        );

        return result;
    }

    // Start
    err = esp_ping_start(ping);

    if (err != ESP_OK) {
        USBSerial.printf(
            "Failed to start ping: %s\n",
            esp_err_to_name(err)
        );

        esp_ping_delete_session(ping);

        return result;
    }

    // Wait until ping finishes
    while (!result.finished) {
        delay(10);
    }

    // Get final statistics
    uint32_t sent = 0;
    uint32_t received = 0;

    esp_ping_get_profile(
        ping,
        ESP_PING_PROF_REQUEST,
        &sent,
        sizeof(sent)
    );

    esp_ping_get_profile(
        ping,
        ESP_PING_PROF_REPLY,
        &received,
        sizeof(received)
    );

    result.sent = sent;
    result.received = received;
    result.lost = sent - received;

    if (result.sent > 0) {
        result.packetLoss =
            (
                (float)result.lost /
                (float)result.sent
            ) * 100.0f;
    } else {
        result.packetLoss = 0.0f;
    }

    if (result.received == 0) {
        result.minRtt = 0;
        result.avgRtt = 0.0f;
        result.maxRtt = 0;
    }

    // Delete session
    esp_ping_delete_session(ping);

    // Result
    USBSerial.printf("\n");

    USBSerial.printf(
        "===== PING RESULT =====\n"
    );

    USBSerial.printf(
        "Target      : %s\n",
        ipString.c_str()
    );

    USBSerial.printf(
        "Sent        : %d\n",
        result.sent
    );

    USBSerial.printf(
        "Received    : %d\n",
        result.received
    );

    USBSerial.printf(
        "Lost        : %d\n",
        result.lost
    );

    USBSerial.printf(
        "Packet Loss : %.1f %%\n",
        result.packetLoss
    );

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
    } else {
        USBSerial.printf(
            "RTT         : N/A\n"
        );
    }

    USBSerial.printf(
        "=======================\n"
    );

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