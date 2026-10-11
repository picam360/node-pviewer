PORT=$1

echo -n "$PORT-0044" | sudo tee /sys/bus/i2c/drivers/e-con_cam/unbind

sleep 0.1

echo -n "$PORT-0040" | sudo tee /sys/bus/i2c/drivers/v-link-ser/unbind

sleep 0.1

echo -n "$PORT-004c" | sudo tee /sys/bus/i2c/drivers/v-link-deser/unbind

sleep 0.1
