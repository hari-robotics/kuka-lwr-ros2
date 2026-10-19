# Development tests

Source packages contain no tests. All migration tests live outside `src/` in
[`../migration_tests/`](../migration_tests/README.md) (marked `COLCON_IGNORE`, never
built or installed). They only observe the packages from outside: they parse or
start launch files and read node parameters; they do not import package code.

```bash
# static launch equivalence against the ROS 1 (lab kuka-lwr) launch files (no ROS 2 needed)
bash ../migration_tests/run_static_checks.sh

# runtime launch equivalence, after colcon build && source install/setup.bash
cd ../migration_tests/single_lwr_launch
python3 test_launch_equivalence.py --expected expected_single_lwr_tool_display.json \
    --mapping mapping_single_lwr_tool_display.json \
    --launch single_lwr_launch single_lwr_tool.launch.py --launch-arg use_lwr_sim:=false
```
