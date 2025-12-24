# Code Explanation

This document explains the key code components of the EKF localization system.

## EKF Core (`ekf_core.py`)

The EKF implementation is in `ekf_localization_tb3/ekf_core.py`. This file contains the mathematical filter logic.

### State Vector and Matrices

```python
class EKFCore:
    def __init__(self, ...):
        self.x = initial_state  # State: [x, y, theta]
        self.P = initial_covariance  # 3x3 covariance matrix
        self.Q = process_noise  # Motion model uncertainty
        self.R_imu = imu_noise  # IMU measurement noise
        self.R_gps = gps_noise  # GPS measurement noise

        # Observation matrices
        self.H_imu = np.array([[0.0, 0.0, 1.0]])  # Observe theta
        self.H_gps = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])  # Observe x, y
```

**Explanation:**
- `x` holds the state estimate [x position, y position, heading angle]
- `P` is the 3x3 covariance matrix representing uncertainty
- `Q` is process noise (motion model uncertainty)
- `R_imu` and `R_gps` are measurement noise matrices
- `H_imu` and `H_gps` are observation matrices that map state to measurements

### Prediction Step

```python
def predict(self, v: float, omega: float, dt: float):
    theta = self.x[2]

    # Motion model: x += v*cos(θ)*dt, y += v*sin(θ)*dt, θ += ω*dt
    dx = np.array([
        v * np.cos(theta) * dt,
        v * np.sin(theta) * dt,
        omega * dt
    ])
    self.x = self.x + dx

    # Jacobian of motion model
    F = np.array([
        [1.0, 0.0, -v * np.sin(theta) * dt],
        [0.0, 1.0, v * np.cos(theta) * dt],
        [0.0, 0.0, 1.0]
    ])
    self.P = F @ self.P @ F.T + self.Q
```

**Explanation:**
1. Extract current heading angle theta
2. Apply velocity motion model to update state
3. Compute Jacobian matrix F (partial derivatives of motion model)
4. Update covariance: P = F * P * F^T + Q

The motion model converts linear velocity `v` and angular velocity `omega` into position changes using basic kinematics.

### IMU Update Step

```python
def update_imu(self, z_theta: float):
    z = np.array([z_theta])
    y = z - self.H_imu @ self.x  # Innovation (measurement - prediction)
    
    S = self.H_imu @ self.P @ self.H_imu.T + self.R_imu  # Innovation covariance
    K = self.P @ self.H_imu.T @ np.linalg.inv(S)  # Kalman gain

    self.x = self.x + (K @ y).flatten()  # Update state
    self.P = (np.eye(3) - K @ self.H_imu) @ self.P  # Update covariance
```

**Explanation:**
1. `y` is the innovation (difference between measurement and prediction)
2. `S` is the innovation covariance
3. `K` is the Kalman gain (determines how much to trust the measurement)
4. State and covariance are updated using standard Kalman filter equations

### GPS Update Step

```python
def update_gps(self, z_x: float, z_y: float):
    z = np.array([z_x, z_y])
    y = z - self.H_gps @ self.x

    S = self.H_gps @ self.P @ self.H_gps.T + self.R_gps
    K = self.P @ self.H_gps.T @ np.linalg.inv(S)

    self.x = self.x + K @ y
    self.P = (np.eye(3) - K @ self.H_gps) @ self.P
```

**Explanation:**
Same structure as IMU update, but observes x and y position instead of theta.

---

## ROS 2 Node (`ekf_node.py`)

The ROS 2 integration is in `ekf_localization_tb3/ekf_node.py`.

### Node Initialization

```python
class EKFLocalizationNode(Node):
    def __init__(self):
        super().__init__('ekf_localization_node')
        self._declare_parameters()
        self._init_ekf()
        self._init_subscribers()
        self._init_publishers()

        self.dt = 1.0 / self.get_parameter('prediction_rate').value
        self.prediction_timer = self.create_timer(self.dt, self._prediction_callback)
```

**Explanation:**
1. Declare ROS parameters for noise values
2. Initialize the EKF with configured parameters
3. Set up subscribers for sensor data
4. Set up publisher for pose output
5. Create timer for prediction step (runs at 50 Hz)

### Velocity Command Callback

```python
def _cmd_vel_callback(self, msg: TwistStamped):
    self.current_v = msg.twist.linear.x
    self.current_omega = msg.twist.angular.z
```

**Explanation:**
Stores the latest velocity commands. These are used in the prediction step.

### Prediction Timer Callback

```python
def _prediction_callback(self):
    self.ekf.predict(self.current_v, self.current_omega, self.dt)
```

**Explanation:**
Runs at 50 Hz. Uses stored velocity commands to propagate state forward in time.

### GPS Callback with Coordinate Conversion

```python
def _gps_callback(self, msg: NavSatFix):
    if msg.status.status < 0:
        return  # Invalid GPS fix

    lat, lon = msg.latitude, msg.longitude

    if not self.gps_initialized:
        self.gps_origin = (lat, lon)  # First reading becomes origin
        self.gps_initialized = True
        return

    x, y = self._latlon_to_local(lat, lon)
    self.ekf.update_gps(x, y)

def _latlon_to_local(self, lat: float, lon: float):
    lat0, lon0 = self.gps_origin
    R = 6371000.0  # Earth radius in meters

    x = R * (lon_rad - lon0_rad) * np.cos(lat0_rad)
    y = R * (lat_rad - lat0_rad)
    return x, y
```

**Explanation:**
1. Skip invalid GPS readings
2. First reading sets the local coordinate origin
3. Convert lat/lon to local x/y using equirectangular projection
4. Call EKF GPS update with local coordinates

### IMU Callback

```python
def _imu_callback(self, msg: Imu):
    q = msg.orientation
    _, _, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
    self.ekf.update_imu(yaw)
    self._publish_pose(msg.header.stamp)
```

**Explanation:**
1. Extract quaternion from IMU message
2. Convert to yaw angle
3. Update EKF with yaw measurement
4. Publish updated pose

---

## Launch File (`ekf_localization.launch.py`)

```python
# Start Gazebo simulation
gz_sim = IncludeLaunchDescription(...)

# Spawn TurtleBot3 with GPS sensor
spawn_robot = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=['-world', 'empty_gps', '-file', urdf_path, ...]
)

# Bridge Gazebo topics to ROS
bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        '/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
        '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        '/cmd_vel@geometry_msgs/msg/TwistStamped]gz.msgs.Twist',
        ...
    ]
)

# Start EKF node
ekf_node = Node(
    package='ekf_localization_tb3',
    executable='ekf_localization_node',
    parameters=[ekf_params_path]
)
```

**Explanation:**
1. Launch Gazebo with GPS-enabled world
2. Spawn robot model with GPS sensor
3. Bridge sensor topics from Gazebo to ROS
4. Start EKF node with configured parameters

---

## Configuration (`ekf_params.yaml`)

```yaml
ekf_localization_node:
  ros__parameters:
    process_noise_x: 0.01      # Motion model uncertainty
    process_noise_y: 0.01
    process_noise_theta: 0.01
    
    imu_noise_theta: 0.001     # IMU measurement noise (rad²)
    
    gps_noise_x: 0.0001        # GPS measurement noise (m²)
    gps_noise_y: 0.0001        # (0.01m)² = 1cm accuracy
    
    prediction_rate: 50.0      # Prediction frequency (Hz)
```

**Explanation:**
- Lower noise values = higher trust in that sensor
- GPS noise set for RTK-like accuracy (~1 cm)
- Higher prediction rate = smoother output

---

## GPS Sensor Model (`model.sdf`)

```xml
<sensor name="navsat" type="navsat">
  <always_on>1</always_on>
  <topic>/navsat</topic>
  <update_rate>10</update_rate>
  <navsat>
    <horizontal_position_stdev>0.0000001</horizontal_position_stdev>
    <vertical_position_stdev>0.0000003</vertical_position_stdev>
  </navsat>
</sensor>
```

**Explanation:**
- NavSat sensor attached to robot
- Publishes at 10 Hz
- Noise configured for RTK-like centimeter accuracy
- 0.0000001° ≈ 1.1 cm horizontal accuracy

---

## Data Logger (`data_logger.py`)

```python
def odom_callback(self, msg: Odometry):
    # First reading becomes origin (zero-referenced)
    if not self.odom_initialized:
        self.odom_origin = (raw_x, raw_y, raw_theta)
        self.odom_initialized = True
    
    # Store relative to origin
    self.latest_gt_x = raw_x - self.odom_origin[0]
    self.latest_gt_y = raw_y - self.odom_origin[1]

def ekf_callback(self, msg: PoseWithCovarianceStamped):
    # Log all data synchronized on EKF publish
    self.csv_writer.writerow([
        elapsed, gt_x, gt_y, gt_theta,
        ekf_x, ekf_y, ekf_theta,
        cov_x, cov_y, cov_theta,
        gps_x, gps_y, imu_theta
    ])
```

**Explanation:**
1. Zero-reference ground truth (first reading = origin)
2. Synchronize logging on EKF publish events
3. Save all data to CSV for later analysis
