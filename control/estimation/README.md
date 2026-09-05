# State Estimation

Estimator implementations convert timestamped sensor measurements into `swp_robot_state_t`.

Initial scope includes a complementary-filter attitude estimator. The boundary is intentionally stable so that observers or Kalman-family estimators can be introduced without changing acquisition, controller, or board APIs.
