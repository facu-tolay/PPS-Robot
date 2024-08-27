mosquitto_pub -h 192.168.1.83 -t "topic/setpoint/ROB_A" -m '{"distance": 1.25,"vx": 0.00,"vy":-0.3,"vr":0.00}'
sleep 12
mosquitto_pub -h 192.168.1.83 -t "topic/setpoint/ROB_A" -m '{"distance": 0.775,"vx": 0.00,"vy":0.00,"vr":-15.00}'
sleep 8
mosquitto_pub -h 192.168.1.83 -t "topic/setpoint/ROB_A" -m '{"distance": 1.25,"vx": 0.00,"vy":-0.3,"vr":0.00}'
sleep 12
mosquitto_pub -h 192.168.1.83 -t "topic/setpoint/ROB_A" -m '{"distance": 0.775,"vx": 0.00,"vy":0.00,"vr":-15.00}'
sleep 8
mosquitto_pub -h 192.168.1.83 -t "topic/setpoint/ROB_A" -m '{"distance": 1.25,"vx": 0.00,"vy":-0.3,"vr":0.00}'
sleep 12
mosquitto_pub -h 192.168.1.83 -t "topic/setpoint/ROB_A" -m '{"distance": 0.775,"vx": 0.00,"vy":0.00,"vr":-15.00}'
