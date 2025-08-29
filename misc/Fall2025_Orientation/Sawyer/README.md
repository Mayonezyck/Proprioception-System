# Sawyer LLM Demo

This repository documents how to set up and run the Sawyer demo, where a Large Language Model (LLM) drives the Rethink Sawyer robot via ROS and a custom API server.

---

## 1. Physical & Network Setup

1. Boot Sawyer into **SDK mode**.  
2. Connect Sawyer’s **Ethernet port** directly to your workstation.  
3. Determine the robot’s IP address (`192.168.1.9` in our setup).  
   - You can check in your router portal, or configure mDNS so `SAWYER.local` resolves correctly.  
4. Verify connectivity:
   ```bash
   ping 192.168.1.9
   ```
5. (Optional) Add a manual alias in `/etc/hosts` if mDNS resolution fails:
   ```
   192.168.1.9   SAWYER SAWYER.local
   ```

---

## 2. Docker & ROS Environment

We run ROS inside a **ROS Melodic Docker container**.

```bash
docker run -it --net=host \
    -v ~/ros_sawyer_ws:/root/ros_sawyer_ws \
    ros:melodic
```

Inside the container:

1. Install dependencies (`cv_bridge`, `control_msgs`, `tf`, etc.).  
2. Build the Intera SDK (`intera_sdk`, `intera_common`, `sawyer_robot`) from source.  
3. Checkout versions consistent with Sawyer’s installed software (5.2.0/5.2.1).  

---

## 3. Environment Configuration (`intera.sh`)

Edit `intera.sh` in the workspace root:

```bash
robot_hostname="192.168.1.9"
your_ip="192.168.1.4"
ros_version="melodic"
```

Then source it:

```bash
bash intera.sh
```

Make sure the following are correct:

- `ROS_MASTER_URI=http://192.168.1.9:11311` (Sawyer is ROS master).  
- `ROS_IP=192.168.1.4` (your PC’s reachable IP).  

---

## 4. ROS Master & Topics

- **No need to run `roscore` locally** — Sawyer provides its own master.  
- Once connected, check topics:
  ```bash
  rostopic list
  ```
- If you see topics but no data, double-check hostname resolution (`SAWYER.local`).  

---

## 5. Enabling the Robot

Enable motion with:

```bash
rosrun intera_interface enable_robot.py -e
```

Common fixes:
- Align SDK source version with robot’s installed software.  
- Patch mismatched imports (`AssemblyState` vs `RobotAssemblyState`).  

---

## 6. Functionality Tests

- **Joint states:**
  ```bash
  rostopic echo /robot/joint_states
  ```

- **Motion control:**
  ```bash
  rosrun intera_examples joint_position_keyboard.py
  ```
  (Expect jerky steps.)  

- **Waypoints:** use `waypoints_joint.py`, but smooth trajectories to avoid “collision detected” errors.  
  - Reduce speed:
    ```bash
    rosparam set /robot/joint_position_speed 0.1
    ```
  - Use small, frequent deltas at ~100 Hz for smoother control.  

---

## 7. Cameras

Camera topics stream under `/io/internal_camera/.../image_raw`.  
Options to visualize:
- Run `web_video_server` and open in browser.  
- Use `image_saver` to dump frames to disk.  

---

## 8. General Fixes

- Install missing Python ROS tools (e.g. `rospy_message_converter`).  
- Disable firewalls/VPNs that block peer-to-peer ROS traffic.  
- Tune **speed ratios**, **payloads**, and **trajectory timing** for smooth motion.  

---

## 9. Running the LLM Demo

The Sawyer demo links the robot to an LLM through a custom **API server**.  

### Step 1: Start Docker & Source Environment
```bash
# Terminal 1
docker exec -it <container> bash
bash intera.sh
```

### Step 2: Start the Face Display
```bash
python face.py
```
- Shows a face animation on Sawyer’s screen.  
- Subscribes to `/is_talking` to switch between idle and talking face animations.  

### Step 3: Start the Actions API Server
```bash
# Terminal 2 (same container)
bash intera.sh
python actions_server.py
```
- Exposes API endpoints to accept commands from the LLM server.  

### Step 4: Start the LLM Output Parser (outside container)
```bash
# On host machine
conda activate ollama-llm
python parseoutput_sawyer_tts.py
```

This script handles LLM outputs, parses them into robot commands, and drives Sawyer via the API server.  

---

## 10. Summary

- Boot Sawyer in SDK mode.  
- Run Docker container with ROS Melodic + Intera SDK.  
- Source `intera.sh` for network config.  
- Start **`face.py`** (robot display) and **`actions_server.py`** (API server).  
- Run **`parseoutput_sawyer_tts.py`** (LLM bridge) outside the container.  

At this point, Sawyer should respond to LLM-generated commands in real time. 🚀
