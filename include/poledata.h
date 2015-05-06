// we are currently only interested in 3D position of the blobs
#define DIM 3

/** data structure that holds the position of the blobs */
typedef struct
{
  // position of the upper blob in m
  double pos_upper[DIM];
  // position of the lower blob in m
  double pos_lower[DIM];
  // anlew of the pole in rad
  double pole_angle;
  // angular velocity of pole [rad/s]
  double pole_angular_velocity;
} PoleData;
