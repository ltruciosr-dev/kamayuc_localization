# TO-DO Tasks

## Simulation Task

- [ ] Add covariance matrix to /wheel_odom (TwistStamped) and publish in topic /leo_odom (TwistWithCovarianceStamped).
- [ ] Test accuracy with `marker<x>_odom` with x: 1 to 15.

Translation between map -> odom seems to have many errors:
- [ ] Add a pose to ekf_global that set zero translation between map - odom
- [ ] /ekf_global/map should only defines a translation between map - odom