# Telemetry Bridge (ROS2 Node)

This script is a bridge between the high-frequency lock-free UDP telemetry stream coming from `simRocket` and the ROS2 ecosystem. 
It receives binary UDP packets on port `9876`, parses them, and publishes standard ROS2 messages (like `geometry_msgs/PoseStamped` and `tf2`).

## How to run without installing ROS2 locally

Since the project uses CMake instead of `colcon`/`ament_cmake` to avoid ROS2 dependencies in the core, you can run this python node using a ROS2 Docker container.

### Step 1: Zbuduj lokalny obraz Dockera (Wykonaj tylko raz!)
Zamiast za każdym razem pobierać i instalować biblioteki (np. MCAP), stworzyliśmy mały `Dockerfile`. Zbuduj obraz jednym poleceniem (będąc w głównym katalogu `simRocket`):
```bash
docker build -t simrocket-ros2 -f rocket_sil_framework/viz_node/Dockerfile .
```

### Step 2: Uruchom mostek telemetrii
Uruchom mostek używając zbudowanego obrazu. 
**Krytyczne:** Używamy flagi `--ipc=host` wraz z `--net=host`, ponieważ ROS2 FastDDS używa pamięci współdzielonej (Shared Memory) do przesyłania wiadomości. Bez `--ipc=host`, dwa kontenery Dockera widzą swoje tematy, ale wiadomości są gubione!
```bash
docker run -it --rm --net=host --ipc=host -v $(pwd)/rocket_sil_framework/viz_node:/viz simrocket-ros2 python3 /viz/telemetry_bridge.py
```

### Step 3: Nagrywanie baga (MCAP)
W osobnym terminalu odpal nagrywanie. Zamiast opcji `-a` (wszystkie tematy, co nagrywa też "szum" startowy jak `/rosout` powodując pusty czas w bagu), nagrywamy tylko nasze tematy:
```bash
docker run -it --rm --net=host --ipc=host -v $(pwd)/bags:/bags simrocket-ros2 bash -c "cd /bags && ros2 bag record /rocket/pose /tf -s mcap"
```

### Step 4: Uruchom symulację
Na samym końcu odpal kod C++ (upewnij się, że terminale z mostkiem i bagiem już działają):
```bash
./build/simRocket
```
