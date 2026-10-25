# Fall detection robot

Development environment for a wheeled TurtleBot with an OAK-D camera.

```text
Windows: Isaac Sim + ROS 2 Bridge (NVIDIA GPU)
                  ↕ Fast DDS / UDP
Linux dev container: ROS 2 Jazzy application nodes
  RGB → YOLO Pose → person tracking → pose sequences → small LSTM → alert
                             ↑ optional aligned OAK-D depth
```

The repository includes the development environment and a small
[TurtleBot3 motion test](docs/turtlebot3-motion.md) with a Windows Isaac Sim scene
script. It does not yet include the detection nodes, trained model, or real
TurtleBot drivers. The real robot's ROS distribution and exact OAK-D variant
still need to be checked before selecting those drivers.

## Prerequisites

- VS Code with the **Dev Containers** extension.
- Docker with Compose **2.24 or newer**, running Linux containers.
- For the Windows quick-start: Docker Desktop **4.34 or newer**, using WSL2,
  with **Settings → Resources → Network → Enable host networking** enabled.
  Enhanced Container Isolation must be off for this networking mode.
- A separately installed Windows Isaac Sim release supporting the **Jazzy**
  ROS bridge. The commands below follow the **Isaac Sim 6.2** library layout;
  check your release's documentation if using another version.
- Optional CUDA inference: a supported Windows NVIDIA driver and working
  Docker Desktop WSL2 GPU passthrough. Do not install a Linux NVIDIA display
  driver inside the container.

**Memory:** the container defaults to an **8 GB limit for ROS/application nodes**,
not for Isaac Sim. Isaac Sim's current documented minimum is **32 GB system
RAM**. This machine has approximately 16 GB total RAM, so running both can cause
memory pressure or fail. The limit is a ceiling, not an allocation or reservation.

## Start the dev container

From the repository root in PowerShell:

```powershell
if (!(Test-Path .devcontainer/.env)) { Copy-Item .env.example .devcontainer/.env }
code .
```

Do not overwrite an existing `.devcontainer/.env` without reviewing it. On a
machine with Docker Engine **inside WSL**, follow the routed setup below instead
of the Desktop localhost defaults, and open VS Code through its **WSL** extension
so Dev Containers uses that Engine. A local, Git-ignored `.env` may already have
been generated for that machine; its IP addresses are not portable.

Review `.devcontainer/.env`, then run **Dev Containers: Reopen in Container**.
The default configuration is CPU-only. If VS Code offers multiple configurations,
choose **Fall detection — ROS 2 Jazzy (CPU)**. The GPU configuration is also
available under `.devcontainer/gpu/devcontainer.json`.

The first build installs ROS 2, `colcon`, `rosdep`, C++/Python development tools,
image transport, `cv_bridge`, and demo nodes. The post-create step updates the
rosdep database; it needs Internet access. Interactive shells source ROS and an
existing workspace overlay automatically. Simulator software is not installed.

Without VS Code, from the repository root:

```powershell
docker compose --env-file .devcontainer/.env -f .devcontainer/compose.yaml up -d --build
docker compose --env-file .devcontainer/.env -f .devcontainer/compose.yaml exec ros bash .devcontainer/scripts/setup-workspace.sh
docker compose --env-file .devcontainer/.env -f .devcontainer/compose.yaml exec ros bash
```

When changing `.devcontainer/.env`, **recreate/rebuild the container**; restarting
an existing container does not update its environment. Avoid changing ROS domain
or DDS settings in only one shell.

## Windows ↔ container communication

ROS messages use **DDS over UDP**, not volumes. Both sides must use Jazzy,
`rmw_fastrtps_cpp`, the same `ROS_DOMAIN_ID`, and reachable advertised IPv4
addresses. The generated Fast DDS profile disables built-in shared-memory
transport and multicast discovery and uses explicit unicast peers. It also adds
the local address as a peer so container nodes discover one another.

### Docker Desktop quick-start (validate on your machine)

The supplied defaults use `127.0.0.1` on both sides, relying on Docker Desktop's
opt-in **bidirectional TCP/UDP host networking**. This is **not** Linux host
networking: Docker Desktop forwards at layer 4 and does not expose the Windows
network interfaces inside the container. DDS support cannot be inferred merely
from a successful TCP connection or `ping`.

**This Windows/WSL Docker topology is not a supported workflow in NVIDIA's ROS
installation guide.** Treat these settings as a starting configuration, not a
guarantee. In particular, DDS port collisions or localhost forwarding behavior
can prevent discovery/data delivery. Verify `/clock` and actual image messages
before depending on it. If it fails, use the routed WSL2 setup below rather than
adding arbitrary Docker port mappings. A default Docker bridge is not sufficient.

### Create the Windows Fast DDS profile

Use Windows Python 3.9+ from the repository root (use `py -3` instead of `python`
if that is your Python launcher):

```powershell
python .devcontainer/scripts/make_dds_profile.py --peers 127.0.0.1 --address 127.0.0.1 --output .devcontainer/generated/windows-fastdds.xml
```

If Python is available only in WSL, run this from the repository in WSL instead:

```bash
python3 .devcontainer/scripts/make_dds_profile.py --peers 127.0.0.1 --address 127.0.0.1 --output .devcontainer/generated/windows-fastdds.xml
```

Open a **fresh Windows PowerShell** without another ROS distribution activated:

```powershell
$isaac = 'C:\isaacsim' # change to your installation
$env:ROS_DISTRO = 'jazzy'
$env:RMW_IMPLEMENTATION = 'rmw_fastrtps_cpp'
$env:ROS_DOMAIN_ID = '0'
$env:FASTRTPS_DEFAULT_PROFILES_FILE = (Resolve-Path '.devcontainer/generated/windows-fastdds.xml').Path
$env:FASTDDS_DEFAULT_PROFILES_FILE = $env:FASTRTPS_DEFAULT_PROFILES_FILE
$env:PATH = "$isaac\exts\isaacsim.ros2.core\jazzy\lib;$env:PATH"
& "$isaac\isaac-sim.bat" --/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge
```

Set the path only once in a fresh shell; use the matching library path for your
release. The container generates its own profile at startup in
`/home/ros/.ros/fastdds.xml`. Windows and Linux profiles are separate files.

**If Fast DDS reports `Node 'metatrafficMulticastLocatorList' without content`:**
an older generator emitted an empty element that Fast DDS 2.x rejects. The current
generator omits it; a nonempty `metatrafficUnicastLocatorList` already disables
default multicast discovery. Do not work around this by unsetting the profile:
that can silently switch back to shared memory/multicast defaults.

Regenerate the Linux profile inside the container using its existing addresses:

```bash
python3 .devcontainer/scripts/make_dds_profile.py \
  --peers "$ROS_DDS_PEERS" --address "$ROS_DDS_ADDRESS" \
  --output "$FASTRTPS_DEFAULT_PROFILES_FILE"
ros2 daemon stop
```

Regenerate the Windows profile with the reverse IP pairing shown below (or the
localhost defaults only for the Desktop localhost setup), then **fully close and
relaunch Isaac Sim**. Stop and restart any running ROS commands: existing DDS
participants do not reload XML profiles. If upgrading an existing container,
**rebuild its image once** to update `/usr/local/bin/make-dds-profile`; otherwise
an older image can regenerate the rejected XML at its next startup.

### Routed alternative: Docker Engine inside WSL2

If Desktop localhost forwarding does not carry DDS reliably, run Docker Engine
inside the Ubuntu WSL2 distribution and keep `network_mode: host`. Connect VS Code
to that WSL distribution, then reopen this project in its dev container. Ensure
the Docker CLI actually targets that Engine, not Docker Desktop's daemon.

In WSL, inspect:

```bash
ip -4 route show default
ip -4 -brief address show eth0
```

For WSL's default NAT mode, the gateway is normally the reachable Windows host
address, and `eth0` has the WSL address. Set:

```dotenv
# Example only: replace with your real addresses.
ROS_DDS_PEERS=172.28.32.1
ROS_DDS_ADDRESS=172.28.40.20
```

Generate the Windows profile with **the reverse pairing**:

```powershell
python .devcontainer/scripts/make_dds_profile.py --peers 172.28.40.20 --address 172.28.32.1 --output .devcontainer/generated/windows-fastdds.xml
```

Recreate the container and relaunch Isaac Sim. These must be actual reachable
addresses on the corresponding hosts, not Docker bridge addresses. Recheck them
after WSL restarts. Mirrored WSL networking has different address/routing behavior;
choose and verify reachable addresses instead of copying these NAT examples.

### Firewalls

Use only a trusted/private network. Do not disable the firewall globally or expose
unauthenticated ROS traffic to the Internet. For domain `0`, the supplied profile
probes participant IDs `0..119`, using the standard DDS UDP range `7400..7649`.
For domain `N`, add `250 * N` to both endpoints. Use a low domain ID such as `0`
for this setup; high IDs can collide with ephemeral ports or exceed valid ports.

If needed, in **Administrator PowerShell**, allow the simulator executable,
adjusting its path and the remote address to the actual peer/subnet:

```powershell
New-NetFirewallRule -DisplayName 'Isaac Sim ROS DDS' -Direction Inbound -Action Allow -Protocol UDP -Program 'C:\isaacsim\kit\kit.exe' -LocalPort 7400-7649 -RemoteAddress 172.28.40.20
```

For a WSL Engine, also inspect the WSL Hyper-V firewall. If it blocks inbound DDS:

```powershell
New-NetFirewallHyperVRule -Name 'FallDetection-WSL-DDS' -DisplayName 'Fall detection WSL DDS' -Direction Inbound -VMCreatorId '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -Protocol UDP -LocalPorts 7400-7649 -RemoteAddresses 172.28.32.1
```

Availability of that command depends on the Windows version. Check for existing
block rules too; they override allow rules. Linux firewall rules, if enabled in
WSL, must also permit the same UDP traffic. Do not blindly apply the sample IPs.

## Verify before adding vision workloads

1. In two container terminals, verify local ROS communication:

   ```bash
   ros2 run demo_nodes_cpp talker
   # In the second terminal:
   ros2 run demo_nodes_cpp listener
   ```

2. In Isaac Sim, add **Tools → Robotics → ROS 2 OmniGraphs → Clock** and press
   **Play**. In the container:

   ```bash
   ros2 daemon stop
   ros2 topic echo /clock
   ```

   Changing timestamps confirm Windows → Linux data delivery, not just discovery.

3. Add a ROS 2 camera publisher and a robot command subscriber to your scene.
   Check the actual topic names/types and QoS:

   ```bash
   ros2 topic list -t
   ros2 topic info /camera/image_raw --verbose
   ros2 topic hz /camera/image_raw --qos-reliability best_effort
   # Only with a matching subscriber and a simulated robot:
   ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
   ```

   Confirm the command is received on the simulator side as well. A scene may
   use a different topic, namespace or `TwistStamped` instead of `Twist`. A ROS
   graph with no image data usually indicates routing, firewall, or QoS issues.
   Begin with low image resolution/frame rate; DDS fragmentation can expose
   problems that a small `/clock` message does not.

4. Application nodes should use `use_sim_time:=true` for simulation, but not for
   the real camera unless a suitable `/clock` source exists. Mounts persist code,
   not the simulated ROS graph; publishers must be configured in the scene.

## Optional YOLO / LSTM dependencies and GPU

ML dependencies are **opt-in** so ordinary ROS work does not download PyTorch or
require a GPU. The Python environment `/opt/venv` can see the system ROS packages.

CPU installation, inside the container:

```bash
bash .devcontainer/scripts/setup-ml.sh cpu
python -c 'import torch; from ultralytics import YOLO; print(torch.__version__)'
```

For NVIDIA acceleration, reopen using `.devcontainer/gpu/devcontainer.json`, or
use both Compose files:

```powershell
docker compose --env-file .devcontainer/.env -f .devcontainer/compose.yaml -f .devcontainer/compose.gpu.yaml up -d --build
```

Then, inside the GPU-enabled container:

```bash
nvidia-smi
bash .devcontainer/scripts/setup-ml.sh cuda
python -c 'import torch; assert torch.cuda.is_available(); x = torch.ones(16, device="cuda"); print(torch.cuda.get_device_name(0), (x + x).sum().item())'
```

The script uses CUDA **12.8 PyTorch wheels**, suitable for Blackwell-class GPUs
such as the RTX 5060 Ti. A separate CUDA toolkit is not needed for these prebuilt
wheels. Custom CUDA extensions may need an additional toolkit. Isaac Sim and
container inference **share** GPU compute and VRAM; the GPU setting does not
reserve memory. YOLO inference is the primary GPU candidate; a small LSTM can
usually run on CPU.

Dependencies use bounded version ranges, not a reproducible ML lockfile. Once
the model is selected and tested, freeze the environment. Re-run ML setup after
rebuilding the container; only the download cache persists. Review Ultralytics'
AGPL/enterprise licensing before distributing a product. Model weights are not
downloaded or trained by container creation.

## ROS workspace and mounts

| Host / volume | Container path | Purpose |
| --- | --- | --- |
| Repository bind mount | `/workspaces/fall-detection-robot` | Source, configs and scripts |
| `./data` within repository | `.../data` | Recordings and datasets; Git-ignored |
| `./models` within repository | `.../models` | Model weights; Git-ignored |
| Named `ros-build`, `ros-install`, `ros-log` | `.../ros_ws/{build,install,log}` | Linux build artifacts |
| Named `ros-state` | `/home/ros/.ros` | ROS logs/state and generated DDS profile |
| Named `pip-cache` | `/home/ros/.cache/pip` | ML package download cache |

Build ROS packages added under `ros_ws/src`:

```bash
cd /workspaces/fall-detection-robot/ros_ws
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
# Run colcon under the venv interpreter so Python node entry points can import
# optional ML packages from /opt/venv as well as system ROS packages.
python /usr/bin/colcon build --symlink-install
source install/setup.bash
```

The nested named volumes intentionally keep Linux build artifacts off the
Windows bind mount. Ordinary Windows tools do not see those artifacts. To use a
large external dataset, add a bind mount to `compose.yaml`, for example source
`D:/robot-data` and target `/datasets` (read-only when appropriate). Create the
host directory first and allow access through Docker Desktop if prompted.

For faster builds with Docker Engine in WSL, keep the repository on WSL's Linux
filesystem rather than `/mnt/c`. The dev user has passwordless sudo for tooling;
this is a development environment, not a hardened production image. No Docker
socket, privileged mode, or blanket host-device access is granted.

Stop services without deleting named volumes:

```powershell
docker compose --env-file .devcontainer/.env -f .devcontainer/compose.yaml down
```

Do not add `--volumes` unless you intend to delete persisted build/state/cache
data. Host bind-mounted `data` and `models` remain on disk independently.

## Real TurtleBot / OAK-D

Prefer connecting the camera to the robot's Linux computer and running the
matching DepthAI ROS driver there. Windows USB devices are **not automatically
available** in the container; direct WSL USB access requires a separate USB/IP
and device-permission setup. No USB passthrough is assumed here.

To receive the real robot's DDS topics, change the peer/address configuration
to mutually reachable LAN addresses, update firewall scopes, and confirm the
robot's ROS distribution, message types, QoS and domain. The localhost defaults
are only for same-PC testing. Camera onboard inference depends on the exact
OAK-D generation and supported model conversion, not just the presence of YOLO.

## Configuration tests

The DDS generator has standard-library tests, runnable on Windows or Linux:

```bash
python3 -B .devcontainer/scripts/test_dds_profile.py
```

These tests check generation/validation behavior. When ROS `rclpy` is available,
they also create a loopback DDS participant in domain `42` and assert that the
native Fast DDS parser accepts the profile without falling back to defaults.
The native checks are skipped on ordinary Windows/Python installations without ROS.
They do not test Windows ↔ Linux data delivery.
Actual image build, GPU access and Windows ↔ Linux communication must be tested
on the intended Docker host.

Inside the container, test imports and a real local UDP talker/listener exchange:

```bash
bash .devcontainer/scripts/smoke-test.sh
```

## References

- [Isaac Sim requirements](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html)
- [Isaac Sim ROS 2 setup and Windows/WSL caveats](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html)
- [Docker Desktop host networking and limitations](https://docs.docker.com/engine/network/drivers/host/)
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/)
