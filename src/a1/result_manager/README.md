# a1_result_manager

`a1_result_manager` subscribes to confirmed danger-source detections and writes
the competition result file:

```text
results/detected_danger.json
```

It intentionally does not perform image detection. That work belongs to
`a1_danger_perception`. This package only manages the final, long-lived result
list for the whole exploration run.

## Run

```bash
cd /workspace/SimEnv
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch a1_result_manager result_manager.launch
```

The node subscribes to:

```text
/danger_perception/detections
```

Only detections with:

- `class_name == "danger_red_sphere"`
- `is_valid == true`
- `status` containing `confirmed`
- frame in `world` or `map`

are accepted into the final result list.

## Output

The JSON file uses the required competition format:

```json
{
  "exploration_time": 98.76,
  "detected_danger_sources": [
    {"position": [2.34, -1.56, 0.25]}
  ]
}
```

If the node restarts, it loads the existing result file first, so previously
written danger sources are not forgotten. New detections are merged with
existing results when they are within `merge_distance_m`.
