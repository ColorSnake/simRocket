#!/usr/bin/env python3
import argparse
import subprocess
import time
import os
import signal
import sys

def wait_for_container_log(container_name, search_string, timeout_s=15):
    """Actively waits until the container outputs the specified string in logs."""
    start_time = time.time()
    while time.time() - start_time < timeout_s:
        result = subprocess.run(f"docker logs {container_name}", shell=True, capture_output=True, text=True)
        if search_string in result.stdout or search_string in result.stderr:
            return True
        time.sleep(0.2) # Poll every 200ms
    return False

def run_command(cmd, wait=False):
    if wait:
        return subprocess.run(cmd, shell=True)
    else:
        return subprocess.Popen(cmd, shell=True)

def cleanup():
    print("\n[Orchestrator] Shutting down and cleaning up containers (please wait for bag file to save)...")
    # docker stop sends SIGTERM, which gives rosbag2 time to flush and close the mcap file cleanly
    subprocess.run("docker stop simrocket_recorder", shell=True, stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
    subprocess.run("docker stop simrocket_bridge", shell=True, stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
    print("[Orchestrator] Cleanup complete!")

def signal_handler(sig, frame):
    cleanup()
    sys.exit(0)

def main():
    parser = argparse.ArgumentParser(description='simRocket Orchestrator')
    parser.add_argument('--log-rosbag', action='store_true', help='Automatically start ROS2 containers to record telemetry')
    parser.add_argument('--debug-docker', action='store_true', help='Stream live logs from Docker containers')
    args = parser.parse_args()

    # Register signals so Ctrl+C cleans up containers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        if args.log_rosbag:
            # Ensure the bags folder exists
            os.makedirs('bags', exist_ok=True)
            
            # Generate unique name for the recording
            bag_name = time.strftime("simrocket_%Y_%m_%d-%H_%M_%S")
            bag_full_path = os.path.abspath(os.path.join('bags', bag_name))
            print(f"[Orchestrator] Telemetry will be saved to: {bag_full_path}")

            print("[Orchestrator] Starting telemetry bridge (telemetry_bridge)...")
            # Map whole project to /workspace so it can read config.json, run as current user
            uid_gid = f"{os.getuid()}:{os.getgid()}"
            run_command(f"docker run -d --rm --name simrocket_bridge --user {uid_gid} -e ROS_LOG_DIR=/tmp --net=host --ipc=host -v $(pwd):/workspace -w /workspace simrocket-ros2 python3 rocket_sil_framework/viz_node/telemetry_bridge.py")

            print("[Orchestrator] Starting ROS2 bag recorder...")
            # Pre-defined list of topics for convenience
            topics = [
                "/rocket/odometry",
                "/rocket/acceleration",
                "/rocket/gps",
                "/tf",
                "/rocket/mass",
                "/rocket/inertia",
                "/rocket/thrust",
                "/rocket/aero_forces",
                "/rocket/visuals/body",
                "/rocket/visuals/thrust",
                "/rocket/visuals/aero",
                "/rocket/visuals/wind",
                "/rocket/visuals/cg",
                "/rocket/visuals/cop",
                "/rocket/tvc/cmd_pitch",
                "/rocket/tvc/cmd_yaw",
                "/rocket/tvc/error_pitch",
                "/rocket/tvc/error_yaw"
            ]
            topics_str = " ".join(topics)
            # Use "exec" and run as current user so bag files are owned by the user, not root
            run_command(f"docker run -d --rm --name simrocket_recorder --user {uid_gid} -e ROS_LOG_DIR=/tmp --net=host --ipc=host -v $(pwd)/bags:/bags -w /bags simrocket-ros2 bash -c \"exec ros2 bag record {topics_str} -o {bag_name} -s mcap\"")
            
            # Active waiting (Event-Driven) instead of rigid sleep()
            print("[Orchestrator] Waiting for ROS2 Bridge to open UDP port...")
            if not wait_for_container_log("simrocket_bridge", "Listening on UDP 9876"):
                print("[WARNING] Bridge initialization timed out!")
                
            print("[Orchestrator] Waiting for ROS2 Recorder to subscribe and start recording...")
            if not wait_for_container_log("simrocket_recorder", "Recording..."):
                print("[WARNING] Recorder initialization timed out!")
            
            print("[Orchestrator] Both services are READY. Launching simulation!")
            
            if args.debug_docker:
                print("[Orchestrator] Attaching Docker log streams...")
                run_command("docker logs -f simrocket_bridge")
                run_command("docker logs -f simrocket_recorder")

        print("\n[Orchestrator] Starting simulation (simRocket)!\n" + "="*50)
        
        # Run C++ binary
        subprocess.run("./build/simRocket", shell=True)
        
        print("="*50 + "\n[Orchestrator] Simulation finished.")
        
    finally:
        if args.log_rosbag:
            cleanup()

if __name__ == '__main__':
    # Validate we are in the project root
    if not os.path.exists("build/simRocket"):
        print("[Error] build/simRocket not found. Make sure you run this script from the project root and the project is built.")
        sys.exit(1)
        
    main()
