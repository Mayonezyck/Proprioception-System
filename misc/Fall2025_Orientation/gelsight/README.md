# GelSight Demo — Full Startup Guide (WidowX250)

This README provides a **step-by-step, verbose** procedure to start the GelSight demo on the WidowX250 platform. It includes **exact commands** (with **no modifications** to what you provided), plus detailed context on what each step does and what you should expect to see.

---

## ⚠️ Critical Notes (Read First)

- **Power order matters:** Power up the **WidowX250 robot first by plugging in the power cable**. **Only after** it’s powered should you connect the robot’s **USB** to the computer.  
  - If you connect USB first, the voltage on the USB will **raise an error ping in two of the motors**.

- **Cable quality matters for GelSight Mini:** Connect the GelSight Mini using a **robust enough cable**—specifically, the **native Type-C cable**—to ensure **sufficient bandwidth** for data transmission. Using a weak/low-quality cable can cause unstable streaming or dropped frames.

---

## Hardware Setup

1. **Power the WidowX250**
   - Plug in the **robot power cable**. Confirm status LEDs/indicators look normal.

2. **Connect the data links**
   - **Robot ↔ Computer:** Connect via **USB** only **after** the robot is powered.
   - **GelSight Mini ↔ Computer:** Connect via **USB** (use the **native Type-C cable** for robust bandwidth).
   - (You may also have other **micro-USB** connections in your setup—ensure all data links are firmly seated and routed to avoid strain.)

3. **Verify connections**
   - Confirm the robot is enumerated as a USB device on the host.  
   - Confirm the GelSight Mini camera/board enumerates and is visible to your system (e.g., as a camera or via your sensor software stack).

---

## Software Environment

You will use **four separate terminals**. In **all four terminals**, you will **activate the same Conda environment** first:

```bash
conda activate ros2
```

> Keep this environment active for the duration of the demo. If any terminal is closed or the environment deactivates, re-run `conda activate ros2` before continuing.

---

## Terminal 1 — Launch Robot Driver, Perception, and Visualization

**Command (no modification):**
```bash
ros2 launch interbotix_xsarm_perception xsarm_perception.launch.py robot_model:=wx250 use_pointcloud_tuner_gui:=true use_armtag_tuner_gui:=false
```

**What this does:**
- **Enables all motors** on the WidowX250.
- Starts **essential ROS 2 nodes** for the robot and perception stack.
- Starts the **Intel RealSense camera**.
- Opens **RViz** for visualization.

**What you should see:**
- A launch log in the terminal showing nodes coming up.
- RViz window opens, with the robot model and incoming streams (as configured).
- RealSense topics become active; you may see pointclouds/frames depending on your RViz configuration.

**If something looks off:**
- Double-check the **power→USB** order for the robot.
- Ensure the **GelSight cable** is the robust native Type-C (for its stream; this launch primarily concerns robot & RealSense).

---

## Terminal 2 — Publish TF for GelSight Sensors

Navigate and run (no modification):
```bash
cd Proprioception-System/arms/wx250
python sensorTFbroadcast.py
```

**Why this is needed:**
- The GelSight sensor is **not designed into the launch file**, so its pose (relative to the fingers) is **published by this TF broadcaster**.
- It **posts the TF between the fingers and the sensors**, allowing downstream nodes and RViz to correctly place the sensor frame(s).

**What you should see:**
- Terminal prints from `sensorTFbroadcast.py` indicating TF frames being published.
- In RViz, you should be able to visualize the sensor frame(s) if TF display is enabled.

---

## Terminal 3 — GelSight Sensor Point Clouds

Navigate and run (no modification):
```bash
cd Proprioception-System/arms/wx250/gsrobotics
python gelsight_point.py
```

**What this script does:**
- Starts a visualization pipeline for the GelSight outputs:
  - **Sensor image**
  - **Contact map**
  - **Inferred depth map**
- **Filters** the point clouds and **publishes** them to ROS topics with the **TF of the sensor base**.
- **Aligns** the points to `base_link`.

**ROS topics published:**
- `sensor_pointcloud`
- `global_pointcloud`

**Interactive controls:**
- Press **`q`** → Quit the program.
- Press **`z`** → Clean up the current points.

**What you should see:**
- Terminal logs indicating frames processed and point clouds published.
- In RViz, point clouds should be visible when TF is correct and displays are enabled.

**If you don’t see point clouds:**
- Confirm Terminal 2 TF broadcaster is running.
- Confirm the GelSight cable is robust enough (native Type-C) and the device is streaming.
- Check that RViz has the correct **Fixed Frame** (e.g., `base_link`) and relevant point cloud displays turned on.

---

## Terminal 4 — Touch Script (Trajectory Playback)

Navigate and run (no modification):
```bash
cd Proprioception-System/arms/wx250
python touch.py
```

**Important:**
- This **touch script must be modified every time** because the **trajectories are hard-coded**.
- Ensure the hard-coded trajectory is safe and free of collisions. Verify joint/Cartesian limits and workspace clearance.

**What you should see:**
- Terminal logs for each motion segment/waypoint.
- In RViz, the robot follows the trajectory; the GelSight point clouds should update as contacts occur.

---

## Optional — Free Move / Reset Motors

If you want to freely move the arm around, run (no modification):
```bash
python restartbot.py
```

**What this does:**
- **Shuts down all motors for ~5 seconds**, then powers them back up.
- **Be very careful**: when motors re-engage, the robot may move slightly.

**Safety reminder:**
- **Hold the robot firmly** when you start this command to maintain control and avoid unexpected motion.

---

## Quick Checklist (TL;DR)

1. **Power On Robot First** → then connect robot **USB** to computer.  
2. **Connect GelSight Mini** with a **robust, native Type-C cable**.  
3. Open **4 terminals** and in each run:
   ```bash
   conda activate ros2
   ```
4. **Terminal 1:**
   ```bash
   ros2 launch interbotix_xsarm_perception xsarm_perception.launch.py robot_model:=wx250 use_pointcloud_tuner_gui:=true use_armtag_tuner_gui:=false
   ```
5. **Terminal 2:**
   ```bash
   cd Proprioception-System/arms/wx250
   python sensorTFbroadcast.py
   ```
6. **Terminal 3:**
   ```bash
   cd Proprioception-System/arms/wx250/gsrobotics
   python gelsight_point.py
   ```
   - Keys: **`q`** to quit, **`z`** to clear points.  
   - Publishes: `sensor_pointcloud`, `global_pointcloud`.
7. **Terminal 4:**
   ```bash
   cd Proprioception-System/arms/wx250
   python touch.py
   ```
   - **Modify this script each time** (hard-coded trajectories).
8. **Optional free-move/reset:**
   ```bash
   python restartbot.py
   ```
   - **Hold the robot** while running this.

---

## What “Success” Looks Like

- **RViz** is open (from Terminal 1), showing the **WidowX250** and streaming data.
- **TF frames** for the GelSight sensors are present (Terminal 2).
- **Point clouds** appear in RViz from `sensor_pointcloud` and/or `global_pointcloud` (Terminal 3).
- **Touch motion** plays safely (Terminal 4), and you observe contact-driven point cloud updates.

---

## Common Gotchas

- **Error pings in two motors** right after plugging USB: this usually means the **USB was connected before power**. Unplug USB, **power the robot**, then reconnect USB.
- **No/unstable GelSight stream:** often a **cable issue**. Use the **native Type-C cable** to guarantee bandwidth.
- **No point clouds in RViz:** ensure **Terminal 2** (TF broadcaster) is running, and that RViz **Fixed Frame** matches (`base_link`) with displays enabled.

---

## Shutdown

- Stop scripts in reverse order (Terminal 4 → 3 → 2 → 1).
- Close RViz and stop the ROS launch.
- Unplug USBs **after** stopping software; power down the robot last if needed.

---

*End of README.*
