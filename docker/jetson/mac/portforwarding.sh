sudo sysctl -w net.inet.ip.forwarding=1

sudo pfctl -f - <<'EOF'
set skip on lo0
scrub in all
nat on en0 from 192.168.55.0/24 to any -> (en0)
pass from 192.168.55.0/24 to any keep state
pass out all keep state
EOF

sudo pfctl -e