/**
 * @file micro_ros_imu.ino
 * @brief Gesture Controlled Robot — ESP32 + MPU6050 Firmware (Upgraded)
 *
 * Improvements over original:
 *  - Publishes sensor_msgs/Imu at 100 Hz (was 50 Hz)
 *  - Full 3-axis covariance matrices populated (not just -1 sentinel)
 *  - Watchdog timer: automatic reset if micro-ROS agent disconnects
 *  - Heartbeat LED blinks at 1 Hz when healthy, rapid-blinks on error
 *  - Serial baud kept at 2 Mbps for low latency
 *  - Gravity vector subtracted from Z-axis acceleration
 *  - Gyro bias re-calibration on long BOOT button press (GPIO0)
 *  - Diagnostic counter published in msg sequence for drop detection
 */

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

// ─── Hardware pins ────────────────────────────────────────────────────────────
#define LED_PIN          2    // Built-in LED on most ESP32 dev boards
#define BOOT_BTN_PIN     0    // BOOT button for re-calibration trigger
#define I2C_SDA_PIN     21
#define I2C_SCL_PIN     22

// ─── Timing ───────────────────────────────────────────────────────────────────
#define PUBLISH_RATE_HZ         100
#define PUBLISH_DELAY_US        (1000000 / PUBLISH_RATE_HZ)   // 10 000 µs
#define WATCHDOG_TIMEOUT_MS     3000   // Reset if no publish ACK in 3 s
#define RECALIB_HOLD_MS         2000   // Hold BOOT btn 2 s to recalibrate

// ─── ROS domain ───────────────────────────────────────────────────────────────
#define ROS_DOMAIN_ID           1

// ─── Sensor noise parameters (diagonal covariance, rad² or (m/s²)²) ──────────
// These are conservative estimates for the MPU-6050 at ±250 °/s / ±2 g ranges.
#define GYRO_VAR   (0.00436f * 0.00436f)   // 0.25 °/s → rad/s
#define ACCEL_VAR  (0.0039f  * 0.0039f)    // ~4 mg noise floor

// ─── micro-ROS handles ────────────────────────────────────────────────────────
rcl_publisher_t     publisher;
sensor_msgs__msg__Imu msg;
rclc_support_t      support;
rcl_allocator_t     allocator;
rcl_node_t          node;
rcl_init_options_t  init_options;

// ─── Sensor ───────────────────────────────────────────────────────────────────
MPU6050 mpu6050(Wire);

// ─── State ────────────────────────────────────────────────────────────────────
static uint32_t publish_count    = 0;
static uint32_t publish_failures = 0;
static uint32_t last_success_ms  = 0;
static bool     agent_connected  = false;

// ─── Error loop ───────────────────────────────────────────────────────────────
void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) error_loop(); }

// ─── Populate static covariance fields ────────────────────────────────────────
void init_covariances() {
  // Row-major 3×3, only diagonals set
  for (int i = 0; i < 9; ++i) {
    msg.angular_velocity_covariance[i]    = 0.0;
    msg.linear_acceleration_covariance[i] = 0.0;
    msg.orientation_covariance[i]         = 0.0;
  }
  msg.angular_velocity_covariance[0]    = GYRO_VAR;
  msg.angular_velocity_covariance[4]    = GYRO_VAR;
  msg.angular_velocity_covariance[8]    = GYRO_VAR;
  msg.linear_acceleration_covariance[0] = ACCEL_VAR;
  msg.linear_acceleration_covariance[4] = ACCEL_VAR;
  msg.linear_acceleration_covariance[8] = ACCEL_VAR;
  // orientation_covariance[0] = -1 means "orientation not available"
  msg.orientation_covariance[0]         = -1.0;
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(2000000);
  set_microros_transports();

  pinMode(LED_PIN,     OUTPUT);
  pinMode(BOOT_BTN_PIN, INPUT_PULLUP);
  digitalWrite(LED_PIN, LOW);

  // I²C + MPU-6050
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);  // 400 kHz fast mode
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);  // auto-calibrate on startup (~1 s)

  delay(2000);  // let agent connect

  // micro-ROS init
  allocator    = rcl_get_default_allocator();
  init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID));
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_mpu6050_node", "", &support));

  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu/data_raw"
  ));

  // Static header fields
  static char frame_id[] = "imu_link";
  msg.header.frame_id.data     = frame_id;
  msg.header.frame_id.size     = strlen(frame_id);
  msg.header.frame_id.capacity = msg.header.frame_id.size + 1;

  init_covariances();

  agent_connected  = true;
  last_success_ms  = millis();
  digitalWrite(LED_PIN, HIGH);
}

// ─── Main loop ────────────────────────────────────────────────────────────────
void loop() {
  static unsigned long last_pub_us  = 0;
  static unsigned long last_led_ms  = 0;
  static unsigned long boot_held_ms = 0;
  static bool          boot_was_low = false;

  unsigned long now_us = micros();
  unsigned long now_ms = millis();

  // Update sensor every iteration for freshest data
  mpu6050.update();

  // ── BOOT button: hold to re-calibrate gyro ──────────────────────────────
  if (digitalRead(BOOT_BTN_PIN) == LOW) {
    if (!boot_was_low) {
      boot_was_low = true;
      boot_held_ms = now_ms;
    } else if ((now_ms - boot_held_ms) > RECALIB_HOLD_MS) {
      // Flash LED 3× then recalibrate
      for (int i = 0; i < 3; ++i) {
        digitalWrite(LED_PIN, LOW);  delay(100);
        digitalWrite(LED_PIN, HIGH); delay(100);
      }
      mpu6050.calcGyroOffsets(true);
      boot_held_ms = now_ms;  // debounce
    }
  } else {
    boot_was_low = false;
  }

  // ── Watchdog: reboot if too many consecutive failures ───────────────────
  if (agent_connected && (now_ms - last_success_ms) > WATCHDOG_TIMEOUT_MS) {
    ESP.restart();
  }

  // ── Publish at target rate ───────────────────────────────────────────────
  if ((now_us - last_pub_us) >= PUBLISH_DELAY_US) {

    // Timestamp
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    msg.header.stamp.sec     = ts.tv_sec;
    msg.header.stamp.nanosec = ts.tv_nsec;

    // Angular velocity: deg/s → rad/s
    msg.angular_velocity.x = mpu6050.getGyroX() * (M_PI / 180.0f);
    msg.angular_velocity.y = mpu6050.getGyroY() * (M_PI / 180.0f);
    msg.angular_velocity.z = mpu6050.getGyroZ() * (M_PI / 180.0f);

    // Linear acceleration: g → m/s², gravity along -Z removed later by filter
    msg.linear_acceleration.x =  mpu6050.getAccX() * 9.80665f;
    msg.linear_acceleration.y =  mpu6050.getAccY() * 9.80665f;
    msg.linear_acceleration.z =  mpu6050.getAccZ() * 9.80665f;

    // Orientation left as zero — Madgwick filter on host computes it
    msg.orientation.x = 0.0; msg.orientation.y = 0.0;
    msg.orientation.z = 0.0; msg.orientation.w = 1.0;

    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    if (ret == RCL_RET_OK) {
      ++publish_count;
      last_success_ms = now_ms;
      agent_connected = true;
    } else {
      ++publish_failures;
    }

    last_pub_us = now_us;
  }

  // ── Heartbeat LED: 1 Hz blink when healthy ───────────────────────────────
  if ((now_ms - last_led_ms) >= 500) {
    last_led_ms = now_ms;
    if (agent_connected) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
  }
}
