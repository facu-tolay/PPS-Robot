mosquitto_pub -h 192.168.1.83 -t "topic/setpoint/ROB_A" -m '{"distance": 1.45,"vx": 0.00,"vy":-0.35,"vr":0.00}'
sleep 12
mosquitto_pub -h 192.168.1.83 -t "topic/setpoint/ROB_A" -m '{"distance": 0.36,"vx": 0.00,"vy":0.00,"vr":-20.00}'
sleep 5
mosquitto_pub -h 192.168.1.83 -t "topic/setpoint/ROB_A" -m '{"distance": 1.00,"vx": 0.00,"vy":-0.35,"vr":0.00}'
sleep 10
mosquitto_pub -h 192.168.1.83 -t "topic/setpoint/ROB_A" -m '{"distance": 0.36,"vx": 0.00,"vy":0.00,"vr":-20.00}'
sleep 5
mosquitto_pub -h 192.168.1.83 -t "topic/setpoint/ROB_A" -m '{"distance": 1.00,"vx": 0.00,"vy":-0.35,"vr":0.00}'
sleep 12
mosquitto_pub -h 192.168.1.83 -t "topic/setpoint/ROB_A" -m '{"distance": 0.36,"vx": 0.00,"vy":0.00,"vr":-20.00}'
sleep 5
mosquitto_pub -h 192.168.1.83 -t "topic/setpoint/ROB_A" -m '{"distance": 1.00,"vx": 0.00,"vy":-0.35,"vr":0.00}'
