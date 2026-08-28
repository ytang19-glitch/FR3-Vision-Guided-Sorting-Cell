# Create the FR3 ROS 2 package

This directory is the source folder of the ROS 2 workspace:

```text
ros2_ws/
└── src/
    └── fr3_vision_sorting/   # created by the steps below
```

> Important: run the `ros2 pkg create` command **inside the Docker container**.  
> If your Ubuntu host prints `bash: ros2: command not found`, ROS 2 is not sourced or installed on the host. That is expected for this Docker-based tutorial.

## 1. Clone the repository on Ubuntu

```bash
cd ~

git clone https://github.com/ytang19-glitch/FR3-Vision-Guided-Sorting-Cell.git fr3_vision_sorting

cd ~/fr3_vision_sorting
```

If you already cloned it, update it instead:

```bash
cd ~/fr3_vision_sorting
git pull origin main
```

## 2. Build and start the Docker container

```bash
cd ~/fr3_vision_sorting

xhost +local:docker
docker compose build
docker compose up -d
docker exec -it fr3_vision_sorting bash
```

Your terminal prompt should now show that you are inside the container.

## 3. Confirm ROS 2 works

Run this **inside Docker**:

```bash
source /opt/ros/jazzy/setup.bash
ros2 --help
printenv ROS_DISTRO
```

The last command should print:

```text
jazzy
```

## 4. Create the package

Still inside Docker:

```bash
cd /workspace/ros2_ws/src

ros2 pkg create fr3_vision_sorting \
  --build-type ament_python \
  --dependencies \
  rclpy \
  geometry_msgs \
  sensor_msgs \
  visualization_msgs \
  moveit_msgs \
  shape_msgs \
  tf2_ros \
  tf2_geometry_msgs \
  cv_bridge
```

Because the repository is mounted at `/workspace`, the generated package will also appear on Ubuntu at:

```text
~/fr3_vision_sorting/ros2_ws/src/fr3_vision_sorting/
```

## 5. Build the workspace

Inside Docker:

```bash
cd /workspace/ros2_ws
source /opt/ros/jazzy/setup.bash

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

source install/setup.bash
ros2 pkg list | grep fr3_vision_sorting
```

Expected result:

```text
fr3_vision_sorting
```

## 6. Save the generated package to GitHub

Exit Docker:

```bash
exit
```
Then run on Ubuntu:

if: GitHub no longer accepts your normal GitHub password for Git operations over HTTPS.

If authentication is required, use GitHub CLI:

```bash
gh auth login
```

Choose:

```text
GitHub.com
↓
HTTPS
↓
Login with a web browser
```

GitHub CLI will display a one-time code.

Press:

```text
Enter
```
Your browser will open.

Log into the correct GitHub account and approve access.

Then:

```bash
cd ~/fr3_vision_sorting

git status
git add ros2_ws/src/fr3_vision_sorting ros2_ws/src/README.md
git commit -m "Add FR3 vision sorting ROS 2 package"
git push origin main
```

## Troubleshooting

### `bash: ros2: command not found`

You are probably running the command on the host. Enter the container:

```bash
cd ~/fr3_vision_sorting
docker compose up -d
docker exec -it fr3_vision_sorting bash
source /opt/ros/jazzy/setup.bash
```

### Container does not exist

Create it first:

```bash
cd ~/fr3_vision_sorting
docker compose up -d --build
```

### Package folder is missing on GitHub

Creating files locally does not automatically upload them. Run the Git commands in Step 6.
