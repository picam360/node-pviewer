while true; do
    ssh root@192.168.1.1 "wpa_cli -i phy1-sta0 scan_result"
    ssh root@192.168.1.1 "wpa_cli -i phy1-sta0 status"
    sleep 1
done
