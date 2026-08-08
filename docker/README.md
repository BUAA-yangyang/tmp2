# CPU Docker environment

This directory is independent of the other simulation workspaces. The image
contains ROS Noetic, Gazebo 11, catkin tools, and the packages used by the
mf82 code. Build it from this workspace:

```bash
docker build -f docker/Dockerfile -t simenv-back-pawn:cpu .
docker run --rm -it --name simenv-back-pawn-cpu \
  --network host \
  -v "$(pwd)/SimEnv:/workspace/SimEnv" \
  simenv-back-pawn:cpu bash
```

Inside the container, build the catkin workspace before running the simulation:

```bash
cd /workspace/SimEnv
catkin_make
source /workspace/SimEnv/devel/setup.bash
```

For the mf82 defaults, run `source /workspace/SimEnv/config/mf82.env`
before invoking `/workspace/SimEnv/auto.sh`.
