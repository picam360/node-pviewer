#!/bin/sh

# リモート接続情報
ROUTER_IP="192.168.1.1"
ROUTER_USER="root"
INTERFACE="phy1-sta0"
HYSTERESIS=10  # 切り替えに必要な電波強度の差分 (dBm)

SSH_CMD="ssh ${ROUTER_USER}@${ROUTER_IP}"

# 1. リモートから現在接続中の BSSID と SSID を取得
STATUS=$($SSH_CMD "wpa_cli -i $INTERFACE status" 2>/dev/null)
CURRENT_BSSID=$(echo "$STATUS" | grep "^bssid=" | cut -d'=' -f2 | tr '[:upper:]' '[:lower:]')
CURRENT_SSID=$(echo "$STATUS" | grep "^ssid=" | cut -d'=' -f2)

if [ -z "$CURRENT_BSSID" ] || [ -z "$CURRENT_SSID" ]; then
    echo "Not connected to any BSSID/SSID."
    exit 1
fi

# 2. リモート側でスキャンを実行
$SSH_CMD "iw dev $INTERFACE scan > /dev/null 2>&1"

# scan_results をリモートから取得し、CURRENT_SSID でフィルタ（ヘッダー行は除去される）
SCAN_RESULTS=$($SSH_CMD "wpa_cli -i $INTERFACE scan_results" 2>/dev/null | grep "$CURRENT_SSID")
echo "$SCAN_RESULTS"

# 現在接続中 BSSID の RSSI を取得
CURRENT_RSSI=$(echo "$SCAN_RESULTS" | awk -v bssid="$CURRENT_BSSID" '
    BEGIN { IGNORECASE=1 }
    $1 == bssid { print $3 }
')

if [ -z "$CURRENT_RSSI" ]; then
    echo "Current BSSID ($CURRENT_BSSID) not found in scan_results."
    exit 1
fi

echo "Current BSSID : $CURRENT_BSSID (SSID: $CURRENT_SSID / RSSI: $CURRENT_RSSI dBm)"

# 3. 同一SSIDの中で最高 RSSI のターゲット候補を計算
#    ※ grep 済みのため NR > 2 や SSID 再チェックは不要
BEST_TARGET=$(echo "$SCAN_RESULTS" | awk \
    -v cur_bssid="$CURRENT_BSSID" \
    -v cur_rssi="$CURRENT_RSSI" \
    -v hys="$HYSTERESIS" '
    BEGIN {
        best_rssi = cur_rssi + hys
        best_bssid = ""
    }
    {
        bssid = tolower($1)
        rssi = $3 + 0

        # 条件1: 現在接続中のBSSIDではないこと
        # 条件2: 現在のRSSI + 10dBm 以上強いこと
        if (bssid != cur_bssid && rssi >= best_rssi) {
            best_rssi = rssi
            best_bssid = bssid
        }
    }
    END {
        if (best_bssid != "") {
            print best_bssid "," best_rssi
        }
    }
')

# 4. 切り替え判定と SSH 経由での FT ローミング実行
if [ -n "$BEST_TARGET" ]; then
    TARGET_BSSID=$(echo "$BEST_TARGET" | cut -d',' -f1)
    TARGET_RSSI=$(echo "$BEST_TARGET" | cut -d',' -f2)

    echo "Found better BSSID in '$CURRENT_SSID': $TARGET_BSSID (RSSI: $TARGET_RSSI dBm >= $CURRENT_RSSI + $HYSTERESIS)"
    echo "Roam triggering via FT..."

    $SSH_CMD "wpa_cli -i $INTERFACE bssid 0 $TARGET_BSSID && wpa_cli -i $INTERFACE reassociate"
else
    echo "No better candidate found in '$CURRENT_SSID' (+${HYSTERESIS}dBm threshold)."
fi