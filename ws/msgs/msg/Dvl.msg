float64 timestamp
float64 time                # DVL internal time

float64 vx                  # velocity x (m/s)
float64 vy                  # velocity y (m/s)
float64 vz                  # velocity z (m/s)

float64[9] velocity_covariance  # 3x3 covariance matrix (row-major)

float64 fom                 # figure of merit
float64 altitude            # altitude from seafloor

bool velocity_valid
int32 status
