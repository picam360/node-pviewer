#include "ping/ping_sock.h"
#include "lwip/ip_addr.h"
#include <esp_wifi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

AsyncWebServer server(80);

void handleGetStats(AsyncWebServerRequest *request) {
    
    USBSerial.println("get-stats called");

    // JsonDocument の生成 (ArduinoJson v7ではサイズ指定不要)
    JsonDocument doc;

    // 基本ステータスの設定
    doc["status"] = "ok";
    doc["uptime_sec"] = millis() / 1000;
    doc["free_heap_bytes"] = ESP.getFreeHeap();
    doc["connected_clients"] = WiFi.softAPgetStationNum();

    // "bat_info" オブジェクトを追加して値を設定
    JsonObject batInfo = doc["bat_info"].to<JsonObject>();
    batInfo["soc"] = g_bat_info.bat_soc;
    batInfo["temp"] = g_bat_info.bat_temp;
    batInfo["volt"] = g_bat_info.bat_volt;
    batInfo["curr"] = g_bat_info.bat_curr;
    batInfo["updated_msec"] = g_bat_info.bat_updated_msec;

    // JSONオブジェクトを文字列に変換
    String jsonResponse;
    serializeJson(doc, jsonResponse);

    // レスポンスオブジェクトの作成 (ステータス200, Content-Type: application/json)
    AsyncWebServerResponse *response = request->beginResponse(200, "application/json", jsonResponse);

    // CORSヘッダーの追加（Webブラウザや他ドメインからのJavaScriptアクセスを許可）
    response->addHeader("Access-Control-Allow-Origin", "*");

    // クライアントへ送信
    request->send(response);
}

// /uplink-test で返却する HTML + JavaScript
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="ja">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP32 通信速度テスト</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            background-color: #f4f7f6;
            display: flex;
            justify-content: center;
            align-items: center;
            min-height: 100vh;
            margin: 0;
        }
        .card {
            background: #ffffff;
            padding: 30px;
            border-radius: 12px;
            box-shadow: 0 4px 15px rgba(0,0,0,0.1);
            width: 360px;
            text-align: center;
        }
        h2 { margin-top: 0; color: #333; }
        .form-group { margin: 20px 0; }
        select, button {
            padding: 10px 14px;
            font-size: 15px;
            border-radius: 6px;
            border: 1px solid #ccc;
        }
        button {
            background-color: #007bff;
            color: white;
            border: none;
            cursor: pointer;
            margin-left: 8px;
        }
        button:disabled { background-color: #aaa; cursor: not-allowed; }
        .result-box {
            margin-top: 20px;
            padding: 15px;
            background: #f8f9fa;
            border-radius: 8px;
            border: 1px solid #eee;
        }
        .speed {
            font-size: 32px;
            font-weight: bold;
            color: #28a745;
            margin: 10px 0;
        }
        .info { font-size: 13px; color: #666; margin-top: 4px; }
        .progress-bar-bg {
            width: 100%;
            background-color: #e9ecef;
            border-radius: 4px;
            height: 8px;
            margin-top: 15px;
            overflow: hidden;
        }
        .progress-bar {
            width: 0%;
            height: 100%;
            background-color: #28a745;
            transition: width 0.1s;
        }
    </style>
</head>
<body>
    <div class="card">
        <h2>速度テスト (Downlink)</h2>
        
        <div class="form-group">
            <label for="sizeSelect">サイズ:</label>
            <select id="sizeSelect">
                <option value="64k" selected>64 KB</option>
                <option value="256k">256 KB</option>
                <option value="512k">512 KB</option>
                <option value="1m">1 MB</option>
            </select>
            <button id="startBtn" onclick="startTest()">スタート</button>
        </div>

        <div class="result-box">
            <div>測定速度</div>
            <div class="speed" id="speedText">0.00 bps</div>
            <div class="info" id="statusText">待機中</div>
            <div class="info" id="detailText">受信: 0 KB / 時間: 0.00 s</div>
            <div class="progress-bar-bg">
                <div class="progress-bar" id="progressBar"></div>
            </div>
        </div>
    </div>

    <script>
        async function startTest() {
            const sizeParam = document.getElementById('sizeSelect').value;
            const btn = document.getElementById('startBtn');
            const speedText = document.getElementById('speedText');
            const statusText = document.getElementById('statusText');
            const detailText = document.getElementById('detailText');
            const progressBar = document.getElementById('progressBar');

            // UIの初期化
            btn.disabled = true;
            statusText.textContent = 'データ受信中...';
            speedText.textContent = '---';
            progressBar.style.width = '0%';

            // 目標バイト数の算出（進捗バー用）
            let targetBytes = 64 * 1024;
            if (sizeParam.endsWith('k')) targetBytes = parseInt(sizeParam) * 1024;
            else if (sizeParam.endsWith('m')) targetBytes = parseInt(sizeParam) * 1024 * 1024;

            try {
                const startTime = performance.now();
                // 指定されたサイズパラメータを付けてリクエスト
                const response = await fetch(`/uplink-test-data?size=${sizeParam}`);

                if (!response.ok) throw new Error(`HTTP Error: ${response.status}`);

                const reader = response.body.getReader();
                let receivedBytes = 0;

                // ストリームデータの受領処理ループ
                while (true) {
                    const { done, value } = await reader.read();
                    if (done) break;

                    receivedBytes += value.length;
                    const elapsedSec = (performance.now() - startTime) / 1000;

                    // 速度（bps & Mbps）の計算
                    const bps = Math.round((receivedBytes * 8) / elapsedSec);
                    const mbps = (bps / 1000000).toFixed(2);
                    const kbps = (bps / 1000).toFixed(1);

                    // リアルタイム表示更新
                    if (bps >= 1000000) {
                        speedText.textContent = `${mbps} Mbps`;
                    } else if (bps >= 1000) {
                        speedText.textContent = `${kbps} kbps`;
                    } else {
                        speedText.textContent = `${bps} bps`;
                    }

                    detailText.textContent = `受信: ${(receivedBytes / 1024).toFixed(1)} KB / 時間: ${elapsedSec.toFixed(2)} s`;
                    const percent = Math.min(100, (receivedBytes / targetBytes) * 100);
                    progressBar.style.width = `${percent}%`;
                }

                const totalSec = (performance.now() - startTime) / 1000;
                const finalBps = Math.round((receivedBytes * 8) / totalSec);
                const finalMbps = (finalBps / 1000000).toFixed(2);

                statusText.textContent = '測定完了！';
                speedText.textContent = finalBps >= 1000000 ? `${finalMbps} Mbps` : `${(finalBps/1000).toFixed(1)} kbps`;
                detailText.textContent = `合計: ${(receivedBytes / 1024).toFixed(1)} KB / 時間: ${totalSec.toFixed(2)} s (${finalBps.toLocaleString()} bps)`;
                progressBar.style.width = '100%';

            } catch (err) {
                console.error(err);
                statusText.textContent = '通信エラーが発生しました';
                speedText.textContent = '0 bps';
            } finally {
                btn.disabled = false;
            }
        }
    </script>
</body>
</html>
)rawliteral";

void initWifi()
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
    WiFi.setTxPower(WIFI_POWER_19_5dBm);

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

    esp_wifi_set_ps(WIFI_PS_NONE);
    //server
    server.on("/get-stats", HTTP_GET, handleGetStats);
    server.on("/uplink-test", HTTP_GET, [](AsyncWebServerRequest *request){
        USBSerial.println("uplink-test called");
        request->send_P(200, "text/html", index_html);
    });
    server.on("/uplink-test-data", HTTP_GET, [](AsyncWebServerRequest *request){
        USBSerial.println("uplink-test-data called");

        String sizeStr = request->arg("size");
        sizeStr.toLowerCase();
        
        size_t totalBytes = 1024 * 1024; // デフォルト 1MB
        if (sizeStr.endsWith("k")) totalBytes = sizeStr.substring(0, sizeStr.length() - 1).toInt() * 1024;
        else if (sizeStr.endsWith("m")) totalBytes = sizeStr.substring(0, sizeStr.length() - 1).toInt() * 1024 * 1024;

        // 1. ESP32のヒープ上に大きめの静的データバッファを確保（例: 32KB）
        //    毎回memsetせず、使い回すことでCPU負荷を極限まで下げる
        static uint8_t dummyBuf[32768];
        static bool inited = false;
        if (!inited) {
            memset(dummyBuf, 'A', sizeof(dummyBuf));
            inited = true;
        }

        // 2. レスポンスの生成
        AsyncWebServerResponse *response = request->beginResponse(
            "application/octet-stream",
            totalBytes,
            [totalBytes](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
            if (index >= totalBytes) return 0;
            
            size_t bytesLeft = totalBytes - index;
            size_t len = (bytesLeft < maxLen) ? bytesLeft : maxLen;

            // 3. memcpyで高速にバッファへコピー
            memcpy(buffer, dummyBuf, len);
            return len;
            }
        );

        response->addHeader("Access-Control-Allow-Origin", "*");
        response->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
        request->send(response);
    });

    // HTTPサーバー起動
    server.begin();
    USBSerial.println("HTTP Server started");
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

        if (config.PING_IP != "")
        {
            if (WiFi.status() == WL_CONNECTED)
            {
                network_check_result = checkNetwork(config.PING_IP);
            }
            else
            {
                USBSerial.println("WiFi disconnected - skip ping");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(CHECK_NETWORK_CYCLE));
    }
}

void network_loop()
{
}