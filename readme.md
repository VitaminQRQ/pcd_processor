
**Step 0:** Install the dependencies: `pcl` and `grid-map`.

**Step 1:** Place your `.pcd` file in the `<path-to-pcd-processor>/data` directory.

**Step 2:** Run `catkin build`, then launch the process using `ros launch pcd_processor pcd_processor.launch`.

Files in the `pcd_processor/memo` directory are not used in this project; they simply serve as usage examples for utilizing a grid map converted from a local PCD file. The functions `get_vehicle_rpy` and `get_vehicle_quaternion` can project the vehicle's pose onto uneven terrain.
