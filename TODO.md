# TO-DO Tasks

## Simulation Task

- [ ] Add covariance matrix to /wheel_odom (TwistStamped) and publish in topic /leo_odom (TwistWithCovarianceStamped).
- [ ] Test `visual_odom` with current `local_ekf/odom` and test accuracy (compared with `gazebo_odom` *don't include it on ekf*)
- [ ] Add all the landmarks to [ekf_global](params/ekf_global.yaml) and [map_artag_ekf.cpp](src/map_artag_ekf.cpp), the last should include a vector of publishers.
- [ ] Test accuracy with `marker<x>_odom` with x: 1 to 15.