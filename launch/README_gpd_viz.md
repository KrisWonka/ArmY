# Gazebo + RViz Real Depth Bridge

## Launch Order (zsh)

```bash
source /opt/ros/melodic/setup.zsh
source /home/hiwonder/jetarm/devel/setup.zsh
```

1. Start camera and make sure `/rgbd_cam/depth/points` is publishing.
1. Start GPD:

```bash
roslaunch gpd_ros ur5.launch
```

1. Start visualization bundle (RViz + optional gpd_grasp, Gazebo off by default):

```bash
roslaunch jetarm_ui gpd_viz_gazebo.launch
```

If you only need visualization (no trigger bridge), disable `gpd_grasp`:

```bash
roslaunch jetarm_ui gpd_viz_gazebo.launch start_gpd_grasp:=false
```

If you want to start Gazebo together:

```bash
roslaunch jetarm_ui gpd_viz_gazebo.launch start_gazebo:=true
```

## Minimal Self-check

```bash
rostopic info /rgbd_cam/depth/points
rostopic info /detect_grasps/plot_grasps
```

Pass criteria:

- RViz shows real-time depth cloud from `/rgbd_cam/depth/points`.
- When GPD outputs results, RViz shows grasp markers on `/detect_grasps/plot_grasps`.
