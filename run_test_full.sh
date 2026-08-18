python3 udp_listener_full.py &
PID=$!
sleep 1
python3 run_sim.py test_liquid.json > /dev/null &
wait $PID
killall -9 run_sim.py python3 2>/dev/null || true
