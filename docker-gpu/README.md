# GPU Docker environment

`Dockerfile` mirrors the reference CUDA 12.8 + ROS Noetic environment while
using a workspace-specific image and container name:

```bash
docker build -f docker-gpu/Dockerfile -t simenv-back-pawn:gpu .
docker run --rm -it --gpus all --name simenv-back-pawn-gpu \
  --network host \
  -e DISPLAY="${DISPLAY:-}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$(pwd)/SimEnv:/workspace/SimEnv" \
  simenv-back-pawn:gpu bash
```

The host path mounted at `/workspace/SimEnv` is this workspace's independent
checkout, so generated maps, logs, and results do not share storage with the
reference worktree.
