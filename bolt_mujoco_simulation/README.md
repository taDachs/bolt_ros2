# bolt\_mujoco\_simulation

## Build Instructions

First, download Mujoco

```bash
mkdir -p ~/mujoco-3.1.6
wget https://github.com/google-deepmind/mujoco/releases/download/3.1.6/mujoco-3.1.6-linux-x86_64.tar.gz -O /tmp/mujoco.tar.gz
tar -xzf /tmp/mujoco.tar.gz -C ~/mujoco-3.1.6 --strip-components=1
```

Then build the workspace using `colcon`

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DMUJOCO_DIR=$HOME/mujoco-3.1.6 --packages-select bolt_mujoo_simulation
```

## Running the simulation

```bash
source install/setup.bash
ros2 launch bolt_mujoco_simulation simulation.launch.py world:=world_unconstrained.xml
```

Options for world are
- `world_unconstrained.xml`: no constrained
- `world_pole_constrained.xml`: can only move the body along the z axis
- `world_saggital_constrained.xml`: can only move the body along the sagittal plane (x, z, pitch)
- `world_fixed.xml`: body fixed in place
