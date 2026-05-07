#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>

#include <BasicLinearAlgebra.h>
using namespace BLA;


/// Lab 3 ///
#include <Wire.h>
#include "ICM_20948.h"
#include "SparkFun_VL53L1X.h"
/// Lab3 ///

//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "789d254e-2393-4828-b10b-3952b9ca360b"

#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"

#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"
//////////// BLE UUIDs ////////////

/// Lab 3 ///
#define WIRE_PORT Wire
#define AD0_VAL 1

ICM_20948_I2C myICM;

icm_20948_DMP_data_t dmp_data;
bool dmp_ready = false;
bool yaw_valid = false;

// #define XSHUT 5

// SFEVL53L1X front(Wire, XSHUT);
// SFEVL53L1X right;

SFEVL53L1X front;
/// Lab3 ///

//////////// Global Variables ////////////
BLEService testService(BLE_UUID_TEST_SERVICE);

BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE);

BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT, BLERead | BLENotify);
BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE);

// RX
RobotCommand robot_cmd(":|");

// TX
EString tx_estring_value;
float tx_float_value = 0.0;

long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;
//////////// Global Variables ////////////

unsigned long timeArray[2000];
int timeIndex = 0;

float tempArray[2000];

/// Lab3 Tof Imu Data Global Variables /// 

const int MAX_TOF = 700;    
const int MAX_IMU = 2500;  

// ToF
unsigned long tof_t[MAX_TOF];
int tof_front[MAX_TOF];
int tof_right[MAX_TOF];
int tof_idx = 0;

// IMU (CF)
unsigned long imu_t[MAX_IMU];
float pitch_cf_arr[MAX_IMU];
float roll_cf_arr[MAX_IMU];
int imu_idx = 0;

/// Lab3 Tof Imu Data Global Variables /// 


/// Lab5 Linear PID control and Linear interpolation /// 

const int right_IN1 = 0;
const int right_IN2 = 1;
const int left_IN3 = 2;
const int left_IN4 = 3;

const int PWM_DEADBAND = 40;

int actual_left_pwm = 0;
int actual_right_pwm = 0;

// 最终真正写给电机的最大 PWM
const int PWM_ACTUAL_MAX = 255;

// 控制器输出的 base PWM 最大值
const int PWM_BASE_MAX = 188;

float left_calib = 1.35f;
float right_calib = 1.0f;

// 参数
float kp = 0.0f, ki = 0.0f, kd = 0.0f;
int target_mm = 304;
const int POS_TOL_MM = 20;
unsigned long test_duration_ms = 10000;

// 运行状态
bool flag_pid_active = false;
bool flag_log_active = false;

// 时间和 PID 中间量
unsigned long pid_start_ms = 0;
unsigned long last_pid_ms = 0;

float current_dist_mm = 0.0f;
float error_mm = 0.0f;
float prev_error_mm = 0.0f;
float integral_error = 0.0f;

bool tof_valid = false;
bool tof_pair_valid = false;

unsigned long prev_tof_ms = 0;
unsigned long last_tof_ms = 0;

float prev_tof_dist_mm = 0.0f;
float last_tof_dist_mm = 0.0f;

float est_dist_mm = 0.0f;       // 外推距离
float control_dist_mm = 0.0f;   // 实际用于控制的距离

float tof_slope_mm_per_ms = 0.0f;

float p_term = 0.0f;
float i_term = 0.0f;
float u_cmd = 0.0f;

float d_term = 0.0f;
float d_term_filtered = 0.0f;
float last_d_term_filtered = 0.0f;

#define D_LPF_ALPHA 0.15

const float INTEGRAL_LIMIT = 3000.0f;

// 日志数组
const int MAX_PID_LOG = 1200;

unsigned long log_t[MAX_PID_LOG];
float log_raw_dist[MAX_PID_LOG];
float log_est_dist[MAX_PID_LOG];
float log_err[MAX_PID_LOG];
float log_u[MAX_PID_LOG];
int log_left_pwm[MAX_PID_LOG];
int log_right_pwm[MAX_PID_LOG];
int log_idx = 0;

const int MAX_PID_RATE = 5000;
unsigned long pid_rate_t[MAX_PID_RATE];
int pid_rate_idx = 0;
bool flag_pid_rate_test = false;

/// Lab5 Linear PID control and Linear interpolation ///

/// Lab6 Orientation PID - Prelab BLE Skeleton ///

float kp_ang = 0.0f;
float ki_ang = 0.0f;
float kd_ang = 0.0f;

float setpoint_yaw_deg = 0.0f;
float current_yaw_deg = 0.0f;

float yaw_error_deg = 0.0f;
float prev_yaw_error_deg = 0.0f;
float prev_yaw_measurement_deg = 0.0f;
float integral_yaw_error = 0.0f;
bool yaw_measurement_initialized = false;

float p_term_ang = 0.0f;
float i_term_ang = 0.0f;
float d_term_ang = 0.0f;
float d_term_ang_filtered = 0.0f;
float last_d_term_ang_filtered = 0.0f;

bool flag_orientation_pid = false;
bool flag_orientation_log = false;

unsigned long orientation_start_ms = 0;
unsigned long last_orientation_ms = 0;
unsigned long orientation_test_duration_ms = 5000;

const float YAW_TOL_DEG = 2.0f;
const float ANG_INTEGRAL_LIMIT = 100.0f;

// 日志数组
const int MAX_ANG_LOG = 1200;

unsigned long ang_t[MAX_ANG_LOG];
float ang_yaw[MAX_ANG_LOG];
float ang_setpoint[MAX_ANG_LOG];
float ang_err[MAX_ANG_LOG];
float ang_u[MAX_ANG_LOG];
float ang_i_dbg[MAX_ANG_LOG];
float ang_d_dbg[MAX_ANG_LOG];
int ang_left_pwm[MAX_ANG_LOG];
int ang_right_pwm[MAX_ANG_LOG];
int ang_idx = 0;

float yaw_deg_dmp = 0.0f;
unsigned long yaw_last_update_ms = 0;

const int PWM_TURN_MIN = 100;
const int PWM_TURN_MAX = 130;
const int DRIFT_TURN_PWM_FIXED = 125;
const float DRIFT_TURN_LEAD_DEG = 130.0f;
const float MAP_YAW_TOL_DEG = 3.0f;

#define D_ANG_LPF_ALPHA 0.05
/// Lab6 Orientation PID - Prelab BLE Skeleton ///

/// Lab7 Step Response for drag / momentum ///

const int MAX_STEP_LOG = 500;

bool flag_step_test = false;

unsigned long step_start_ms = 0;
int step_base_pwm_cmd = 130;
int step_stop_dist_mm = 1000;

unsigned long step_t[MAX_STEP_LOG];
float step_tof_mm[MAX_STEP_LOG];
int step_base_pwm_log[MAX_STEP_LOG];
int step_left_pwm_log[MAX_STEP_LOG];
int step_right_pwm_log[MAX_STEP_LOG];
int step_idx = 0;

/// Lab7 Step Response for drag / momentum ///

/// Lab7 Kalman Filter globals ///

// 是否启用 KF 版 PID（后面再真正接进主循环）
bool flag_kf_pid = false;

// KF 是否已经用第一帧 ToF 初始化
bool kf_initialized = false;

// 记录最近一次 KF 更新时刻
unsigned long kf_last_ms = 0;

// 当前 KF 估计结果（方便直接拿来做控制/记录）
float kf_est_dist_mm = 0.0f;   // 正的距离值，给 PID 用
float kf_est_vel_mm_s = 0.0f;  // 速度状态，单位 mm/s

float kf_last_u_scaled = 0.0f; 

// ===== KF matrices =====

Matrix<2,1> kf_mu = {0.0f, 0.0f};

Matrix<2,2> kf_Sigma = {10000.0f, 0.0f,
                        0.0f, 250000.0f};

Matrix<2,2> kf_Ad = {1.0f, 0.03912f,
                     0.0f, 0.93350278f};

Matrix<2,1> kf_Bd = {0.0f,
                     151.664365f};

Matrix<1,2> kf_C = {-1.0f, 0.0f};

Matrix<2,2> kf_Sigma_u = {2500.0f, 0.0f,
                          0.0f, 90000.0f};

Matrix<1,1> kf_Sigma_z = {1600.0f};

/// Lab7 Kalman Filter globals ///

/// Lab7 KF debug log ///
const int MAX_KF_LOG = 1200;

unsigned long kf_log_t[MAX_KF_LOG];
float kf_log_raw_tof[MAX_KF_LOG];
float kf_log_est_dist[MAX_KF_LOG];
float kf_log_est_vel[MAX_KF_LOG];
float kf_log_u_scaled[MAX_KF_LOG];
int kf_log_idx = 0;

/// Lab7 KF debug log ///

/// Lab8 Drift globals ///

bool flag_drift = false;

enum DriftState {
    DRIFT_IDLE = 0,
    DRIFT_FORWARD,
    DRIFT_TURN,
    DRIFT_EXIT,
    DRIFT_DONE
};

DriftState drift_state = DRIFT_IDLE;

// 前进阶段
int drift_forward_pwm = 150;
int drift_trigger_mm = 950;

// 转向阶段
float drift_target_yaw_deg = 0.0f;
bool drift_has_target = false;
float drift_start_yaw_deg = 0.0f;
float drift_last_yaw_deg = 0.0f;
float drift_turn_progress_deg = 0.0f;
float drift_turn_release_progress_deg = 156.0f;
int drift_turn_dir = 0;
int drift_turn_pwm_cmd = 0;

// 转完后的退出直行阶段
int drift_exit_pwm = 150;
unsigned long drift_exit_duration_ms = 400;
unsigned long drift_exit_start_ms = 0;

// 计时
unsigned long drift_start_ms = 0;
unsigned long drift_turn_start_ms = 0;
unsigned long drift_max_runtime_ms = 7000;

const int MAX_DRIFT_LOG = 1400;

unsigned long drift_log_t[MAX_DRIFT_LOG];
int   drift_log_state[MAX_DRIFT_LOG];
float drift_log_raw_tof[MAX_DRIFT_LOG];
float drift_log_yaw[MAX_DRIFT_LOG];
float drift_log_target_yaw[MAX_DRIFT_LOG];
float drift_log_yaw_err[MAX_DRIFT_LOG];
int   drift_log_left_pwm[MAX_DRIFT_LOG];
int   drift_log_right_pwm[MAX_DRIFT_LOG];
int   drift_log_idx = 0;

/// Lab8 Drift globals ///

/// Lab9 Mapping globals ///

bool flag_map_scan = false;

enum MapScanState {
    MAP_SCAN_IDLE = 0,
    MAP_SCAN_SETTLING,
    MAP_SCAN_TURNING,
    MAP_SCAN_DONE
};

MapScanState map_scan_state = MAP_SCAN_IDLE;

float map_scan_step_deg = 20.0f;
int map_scan_num_steps = 18;
unsigned long map_scan_settle_ms = 150;

unsigned long map_scan_start_ms = 0;
unsigned long map_scan_settle_start_ms = 0;

float map_scan_start_yaw_deg = 0.0f;
float map_scan_target_yaw_deg = 0.0f;
int map_scan_sample_idx = 0;

const int MAX_MAP_LOG = 128;

unsigned long map_log_t[MAX_MAP_LOG];
int map_log_step_idx[MAX_MAP_LOG];
float map_log_yaw_deg[MAX_MAP_LOG];
float map_log_dist_mm[MAX_MAP_LOG];
float map_log_setpoint_deg[MAX_MAP_LOG];
int map_log_idx = 0;

//////////// Lab12 Local Planning Variables ////////////
bool flag_nav_turn = false;
bool flag_nav_drive = false;

const int MAX_NAV_LOG = 500;
const int NAV_MODE_TURN = 1;
const int NAV_MODE_DRIVE = 2;
const float NAV_TURN_TOL_DEG = 3.0f;
const unsigned long NAV_TURN_STABLE_MS = 180;

unsigned long nav_start_ms = 0;
unsigned long nav_timeout_ms = 0;
unsigned long nav_turn_stable_since_ms = 0;
float nav_start_yaw_deg = 0.0f;
float nav_target_yaw_deg = 0.0f;
float nav_turn_delta_deg = 0.0f;
float nav_drive_heading_deg = 0.0f;
float nav_heading_kp = 0.0f;
float nav_last_tof_mm = -1.0f;
int nav_drive_base_pwm = 0;
int nav_drive_duration_ms = 0;
int nav_front_stop_mm = 0;
int nav_drive_direction = 1;

unsigned long nav_log_t[MAX_NAV_LOG];
int nav_log_mode[MAX_NAV_LOG];
float nav_log_yaw[MAX_NAV_LOG];
float nav_log_target[MAX_NAV_LOG];
float nav_log_err[MAX_NAV_LOG];
float nav_log_tof[MAX_NAV_LOG];
int nav_log_left_pwm[MAX_NAV_LOG];
int nav_log_right_pwm[MAX_NAV_LOG];
int nav_log_idx = 0;

/// Lab9 Mapping globals ///

enum CommandTypes
{
    PING,
    SEND_TWO_INTS,
    SEND_THREE_FLOATS,
    ECHO,
    DANCE,
    SET_VEL,
    GET_TIME_MILLIS,
    TIME_LOOP,
    SEND_TIME_DATA,
    GET_TEMP_READINGS,
    READ_IMU_TOF,

// 从电脑发 Kp、Ki、Kd
    SET_PID_GAINS,
// 开始一次固定时长测试，并开始记录
    START_PID_LOG,
// 手动停止测试
    STOP_PID_LOG,
// 把记录的数据回传到电脑
    SEND_PID_LOG,
    START_PID_RATE_TEST,
    SEND_PID_RATE_TEST,

    PID_ORIENTATION,
    STOP_ORIENTATION_PID,
    SEND_ANGULAR_DATA,
    SET_ANGULAR_SETPOINT,

    START_STEP_RESPONSE,
    STOP_STEP_RESPONSE,
    SEND_STEP_RESPONSE,

    START_KF_PID_LOG,
    STOP_KF_PID_LOG,
    SEND_KF_LOG,

    START_DRIFT,
    STOP_DRIFT,
    SEND_DRIFT_LOG,

    START_MAP_SCAN,
    STOP_MAP_SCAN,
    SEND_MAP_SCAN,
    TURN_REL_DEG,
    DRIVE_CELL_MM,
    STOP_NAV,
    SEND_NAV_LOG,
};

void stop_motors() {
  actual_left_pwm = 0;
  actual_right_pwm = 0;

  analogWrite(right_IN1, 0);
  analogWrite(right_IN2, 0);
  analogWrite(left_IN3, 0);
  analogWrite(left_IN4, 0);
}

int clamp_pwm_base(int pwm_val) {
    if (pwm_val < 0) return 0;
    if (pwm_val > PWM_BASE_MAX) return PWM_BASE_MAX;
    return pwm_val;
}

int clamp_pwm_turn(int pwm_val) {
    if (pwm_val <= 0) return 0;
    if (pwm_val < PWM_TURN_MIN) return PWM_TURN_MIN;
    if (pwm_val > PWM_TURN_MAX) return PWM_TURN_MAX;
    return pwm_val;
}

void drive_forward_pwm(int pwm_base) {
    pwm_base = clamp_pwm_base(pwm_base);

    if (pwm_base > 0 && pwm_base < PWM_DEADBAND) {
        pwm_base = PWM_DEADBAND;
    }

    int left_pwm  = (int)(pwm_base * left_calib);
    int right_pwm = (int)(pwm_base * right_calib);

    if (left_pwm > PWM_ACTUAL_MAX) left_pwm = PWM_ACTUAL_MAX;
    if (right_pwm > PWM_ACTUAL_MAX) right_pwm = PWM_ACTUAL_MAX;

    actual_left_pwm = left_pwm;
    actual_right_pwm = right_pwm;

    analogWrite(right_IN1, right_pwm);
    analogWrite(right_IN2, 0);

    analogWrite(left_IN3, left_pwm);
    analogWrite(left_IN4, 0);
}

void drive_backward_pwm(int pwm_base) {
    pwm_base = clamp_pwm_base(pwm_base);

    if (pwm_base > 0 && pwm_base < PWM_DEADBAND) {
        pwm_base = PWM_DEADBAND;
    }

    int left_pwm  = (int)(pwm_base * left_calib);
    int right_pwm = (int)(pwm_base * right_calib);

    if (left_pwm > PWM_ACTUAL_MAX) left_pwm = PWM_ACTUAL_MAX;
    if (right_pwm > PWM_ACTUAL_MAX) right_pwm = PWM_ACTUAL_MAX;

    actual_left_pwm = -left_pwm;
    actual_right_pwm = -right_pwm;

    analogWrite(right_IN1, 0);
    analogWrite(right_IN2, right_pwm);

    analogWrite(left_IN3, 0);
    analogWrite(left_IN4, left_pwm);
}

int clamp_drive_side_pwm(int pwm_base) {
    pwm_base = clamp_pwm_base(pwm_base);
    if (pwm_base > 0 && pwm_base < PWM_DEADBAND) {
        pwm_base = PWM_DEADBAND;
    }
    return pwm_base;
}

void drive_forward_split_pwm(int left_base, int right_base) {
    left_base = clamp_drive_side_pwm(left_base);
    right_base = clamp_drive_side_pwm(right_base);

    int left_pwm = (int)(left_base * left_calib);
    int right_pwm = (int)(right_base * right_calib);

    if (left_pwm > PWM_ACTUAL_MAX) left_pwm = PWM_ACTUAL_MAX;
    if (right_pwm > PWM_ACTUAL_MAX) right_pwm = PWM_ACTUAL_MAX;

    actual_left_pwm = left_pwm;
    actual_right_pwm = right_pwm;

    analogWrite(right_IN1, right_pwm);
    analogWrite(right_IN2, 0);

    analogWrite(left_IN3, left_pwm);
    analogWrite(left_IN4, 0);
}

void drive_backward_split_pwm(int left_base, int right_base) {
    left_base = clamp_drive_side_pwm(left_base);
    right_base = clamp_drive_side_pwm(right_base);

    int left_pwm = (int)(left_base * left_calib);
    int right_pwm = (int)(right_base * right_calib);

    if (left_pwm > PWM_ACTUAL_MAX) left_pwm = PWM_ACTUAL_MAX;
    if (right_pwm > PWM_ACTUAL_MAX) right_pwm = PWM_ACTUAL_MAX;

    actual_left_pwm = -left_pwm;
    actual_right_pwm = -right_pwm;

    analogWrite(right_IN1, 0);
    analogWrite(right_IN2, right_pwm);

    analogWrite(left_IN3, 0);
    analogWrite(left_IN4, left_pwm);
}

void turn_left_pwm(int pwm_base) {
    pwm_base = clamp_pwm_turn(pwm_base);

    int left_pwm  = (int)(pwm_base * left_calib);
    int right_pwm = (int)(pwm_base * right_calib);

    if (left_pwm > PWM_ACTUAL_MAX) left_pwm = PWM_ACTUAL_MAX;
    if (right_pwm > PWM_ACTUAL_MAX) right_pwm = PWM_ACTUAL_MAX;

    // 左轮后退，右轮前进
    actual_left_pwm = -left_pwm;
    actual_right_pwm = right_pwm;

    analogWrite(right_IN1, right_pwm);
    analogWrite(right_IN2, 0);

    analogWrite(left_IN3, 0);
    analogWrite(left_IN4, left_pwm);
}

void turn_right_pwm(int pwm_base) {
    pwm_base = clamp_pwm_turn(pwm_base);

    int left_pwm  = (int)(pwm_base * left_calib);
    int right_pwm = (int)(pwm_base * right_calib);

    if (left_pwm > PWM_ACTUAL_MAX) left_pwm = PWM_ACTUAL_MAX;
    if (right_pwm > PWM_ACTUAL_MAX) right_pwm = PWM_ACTUAL_MAX;

    // 左轮前进，右轮后退
    actual_left_pwm = left_pwm;
    actual_right_pwm = -right_pwm;

    analogWrite(right_IN1, 0);
    analogWrite(right_IN2, right_pwm);

    analogWrite(left_IN3, left_pwm);
    analogWrite(left_IN4, 0);
}

// 每次开始新测试前，把上一轮实验留下的状态清掉，不然积分项、误差、时间戳会串掉。
void reset_pid_log() {
    log_idx = 0;

    pid_start_ms = 0;
    last_pid_ms = 0;

    current_dist_mm = 0.0f;
    error_mm = 0.0f;
    prev_error_mm = 0.0f;
    integral_error = 0.0f;

    p_term = 0.0f;
    i_term = 0.0f;
    u_cmd = 0.0f;

    tof_valid = false;
    tof_pair_valid = false;

    prev_tof_ms = 0;
    last_tof_ms = 0;

    prev_tof_dist_mm = 0.0f;
    last_tof_dist_mm = 0.0f;

    est_dist_mm = 0.0f;
    control_dist_mm = 0.0f;
    tof_slope_mm_per_ms = 0.0f;

    d_term = 0;
    d_term_filtered = 0;
    last_d_term_filtered = 0;

    actual_left_pwm = 0;
    actual_right_pwm = 0;
}
// 每次控制循环记录一条数据。
void log_pid_sample(unsigned long t_ms, float raw_mm, float est_mm, float err_mm, float u, int left_pwm, int right_pwm) {
    if (!flag_log_active) return;
    if (log_idx >= MAX_PID_LOG) return;

    log_t[log_idx] = t_ms;
    log_raw_dist[log_idx] = raw_mm;
    log_est_dist[log_idx] = est_mm;
    log_err[log_idx] = err_mm;
    log_u[log_idx] = u;
    log_left_pwm[log_idx] = left_pwm;
    log_right_pwm[log_idx] = right_pwm;

    log_idx++;
}

// 实验结束后，通过 BLE 把数组一条条发回 Python。
void send_pid_log() {
    tx_estring_value.clear();
    tx_estring_value.append("PID_BEGIN");
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
    delay(10);

    for (int i = 0; i < log_idx; i++) {
        tx_estring_value.clear();
        tx_estring_value.append("PID,");
        tx_estring_value.append((int)log_t[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(log_raw_dist[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(log_est_dist[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(log_err[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(log_u[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(log_left_pwm[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(log_right_pwm[i]);
    
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        BLE.poll();
        delay(3);
    }

    tx_estring_value.clear();
    tx_estring_value.append("PID_END,");
    tx_estring_value.append(log_idx);
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
    delay(10);
}

void reset_kf_log() {
    kf_log_idx = 0;
}

void reset_kf_state(float first_tof_mm, unsigned long now_ms) {
    // state: [x; xdot], where x = -distance
    kf_mu(0,0) = -first_tof_mm;
    kf_mu(1,0) = 0.0f;

    // reset covariance to your chosen initial Sigma
    kf_Sigma = {10000.0f, 0.0f,
                0.0f, 250000.0f};

    kf_est_dist_mm = first_tof_mm;
    kf_est_vel_mm_s = 0.0f;

    kf_last_ms = now_ms;
    kf_initialized = true;
}

void log_kf_sample(unsigned long t_ms,
                   float raw_tof_mm,
                   float est_dist_mm,
                   float est_vel_mm_s,
                   float u_scaled) {
    if (kf_log_idx >= MAX_KF_LOG) return;

    kf_log_t[kf_log_idx] = t_ms;
    kf_log_raw_tof[kf_log_idx] = raw_tof_mm;
    kf_log_est_dist[kf_log_idx] = est_dist_mm;
    kf_log_est_vel[kf_log_idx] = est_vel_mm_s;
    kf_log_u_scaled[kf_log_idx] = u_scaled;

    kf_log_idx++;
}

void run_kf_step(float u_scaled, float y_meas_mm, bool do_update) {
    Matrix<2,2> I2 = {1.0f, 0.0f,
                      0.0f, 1.0f};

    Matrix<1,1> u = {u_scaled};
    Matrix<1,1> y = {y_meas_mm};

    Matrix<2,1> mu_p = kf_Ad * kf_mu + kf_Bd * u;
    Matrix<2,2> Sigma_p = kf_Ad * kf_Sigma * ~kf_Ad + kf_Sigma_u;

    if (do_update) {
        Matrix<1,1> Sigma_m = kf_C * Sigma_p * ~kf_C + kf_Sigma_z;

        Invert(Sigma_m);

        Matrix<2,1> K = Sigma_p * ~kf_C * Sigma_m;

        Matrix<1,1> y_m = y - kf_C * mu_p;

        kf_mu = mu_p + K * y_m;
        kf_Sigma = (I2 - K * kf_C) * Sigma_p;
    } else {
        kf_mu = mu_p;
        kf_Sigma = Sigma_p;
    }

    kf_est_dist_mm = -kf_mu(0,0);   
    kf_est_vel_mm_s = kf_mu(1,0);
}

float wrap_angle_deg(float ang) {
    while (ang > 180.0f) ang -= 360.0f;
    while (ang < -180.0f) ang += 360.0f;
    return ang;
}

void reset_orientation_controller_terms() {
    yaw_error_deg = 0.0f;
    prev_yaw_error_deg = 0.0f;
    prev_yaw_measurement_deg = 0.0f;
    integral_yaw_error = 0.0f;
    yaw_measurement_initialized = false;

    p_term_ang = 0.0f;
    i_term_ang = 0.0f;
    d_term_ang = 0.0f;
    d_term_ang_filtered = 0.0f;
    last_d_term_ang_filtered = 0.0f;
}

void reset_orientation_log() {
    ang_idx = 0;

    orientation_start_ms = 0;
    last_orientation_ms = 0;

    current_yaw_deg = 0.0f;

    reset_orientation_controller_terms();

    actual_left_pwm = 0;
    actual_right_pwm = 0;
}

void log_orientation_sample(unsigned long t_ms,
                            float yaw_deg,
                            float setpoint_deg,
                            float err_deg,
                            float u,
                            float i_dbg,
                            float d_dbg,
                            int left_pwm,
                            int right_pwm) {
    if (!flag_orientation_log) return;
    if (ang_idx >= MAX_ANG_LOG) return;

    ang_t[ang_idx] = t_ms;
    ang_yaw[ang_idx] = yaw_deg;
    ang_setpoint[ang_idx] = setpoint_deg;
    ang_err[ang_idx] = err_deg;
    ang_u[ang_idx] = u;
    ang_i_dbg[ang_idx] = i_dbg;
    ang_d_dbg[ang_idx] = d_dbg;
    ang_left_pwm[ang_idx] = left_pwm;
    ang_right_pwm[ang_idx] = right_pwm;

    ang_idx++;
}

void send_angular_data() {
    tx_estring_value.clear();
    tx_estring_value.append("ANG_BEGIN");
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
    delay(10);

    for (int i = 0; i < ang_idx; i++) {
        tx_estring_value.clear();
        tx_estring_value.append("ANG,");
        tx_estring_value.append((int)ang_t[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(ang_yaw[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(ang_setpoint[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(ang_err[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(ang_u[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(ang_i_dbg[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(ang_d_dbg[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(ang_left_pwm[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(ang_right_pwm[i]);

        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        BLE.poll();
        delay(3);
    }

    tx_estring_value.clear();
    tx_estring_value.append("ANG_END,");
    tx_estring_value.append(ang_idx);
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
    delay(10);
}

bool setup_dmp() {
    bool success = true;

    success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);

    success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);

    success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 2) == ICM_20948_Stat_Ok);

    success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
    success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
    success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
    success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

    dmp_ready = success;
    return success;
}

bool update_yaw_from_dmp() {
    if (!dmp_ready) return false;

    myICM.readDMPdataFromFIFO(&dmp_data);

    if (myICM.status == ICM_20948_Stat_FIFONoDataAvail) {
        return false;
    }

    if ((myICM.status == ICM_20948_Stat_Ok) ||
        (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {

        if ((dmp_data.header & DMP_header_bitmap_Quat6) > 0) {

            double q1 = ((double)dmp_data.Quat6.Data.Q1) / 1073741824.0;
            double q2 = ((double)dmp_data.Quat6.Data.Q2) / 1073741824.0;
            double q3 = ((double)dmp_data.Quat6.Data.Q3) / 1073741824.0;

            double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

            double qw = q0;
            double qx = q2;
            double qy = q1;
            double qz = -q3;

            double t3 = +2.0 * (qw * qz + qx * qy);
            double t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
            double yaw = atan2(t3, t4) * 180.0 / PI;

            yaw_deg_dmp = yaw;
            current_yaw_deg = wrap_angle_deg((float)yaw_deg_dmp);

            yaw_valid = true;
            yaw_last_update_ms = millis();
            return true;
        }
    }

    return false;
}

void reset_step_log() {
    step_idx = 0;
    step_start_ms = 0;

    actual_left_pwm = 0;
    actual_right_pwm = 0;
}

void log_step_sample(unsigned long t_ms,
                     float tof_mm,
                     int base_pwm,
                     int left_pwm,
                     int right_pwm) {
    if (!flag_step_test) return;
    if (step_idx >= MAX_STEP_LOG) return;

    step_t[step_idx] = t_ms;
    step_tof_mm[step_idx] = tof_mm;
    step_base_pwm_log[step_idx] = base_pwm;
    step_left_pwm_log[step_idx] = left_pwm;
    step_right_pwm_log[step_idx] = right_pwm;

    step_idx++;
}

void send_step_log() {
    tx_estring_value.clear();
    tx_estring_value.append("STEP_BEGIN");
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
    delay(10);

    for (int i = 0; i < step_idx; i++) {
        tx_estring_value.clear();
        tx_estring_value.append("STEP,");
        tx_estring_value.append((int)step_t[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(step_tof_mm[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(step_base_pwm_log[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(step_left_pwm_log[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(step_right_pwm_log[i]);

        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        BLE.poll();
        delay(3);
    }

    tx_estring_value.clear();
    tx_estring_value.append("STEP_END,");
    tx_estring_value.append(step_idx);
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
    delay(10);
}

void send_kf_log() {
    tx_estring_value.clear();
    tx_estring_value.append("KFPID_BEGIN");
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
    delay(10);

    for (int i = 0; i < kf_log_idx; i++) {
        tx_estring_value.clear();
        tx_estring_value.append("KFPID,");
        tx_estring_value.append((int)kf_log_t[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(kf_log_raw_tof[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(kf_log_est_dist[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(kf_log_est_vel[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(kf_log_u_scaled[i]);

        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        BLE.poll();
        delay(3);
    }

    tx_estring_value.clear();
    tx_estring_value.append("KFPID_END,");
    tx_estring_value.append(kf_log_idx);
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
    delay(10);
}

void reset_drift_log() {
    drift_log_idx = 0;
}

void log_drift_sample(unsigned long t_ms,
                      int state_code,
                      float raw_tof_mm,
                      float yaw_deg,
                      float target_yaw_deg,
                      float yaw_err_deg,
                      int left_pwm,
                      int right_pwm) {
    if (drift_log_idx >= MAX_DRIFT_LOG) return;

    drift_log_t[drift_log_idx] = t_ms;
    drift_log_state[drift_log_idx] = state_code;
    drift_log_raw_tof[drift_log_idx] = raw_tof_mm;
    drift_log_yaw[drift_log_idx] = yaw_deg;
    drift_log_target_yaw[drift_log_idx] = target_yaw_deg;
    drift_log_yaw_err[drift_log_idx] = yaw_err_deg;
    drift_log_left_pwm[drift_log_idx] = left_pwm;
    drift_log_right_pwm[drift_log_idx] = right_pwm;

    drift_log_idx++;
}

void send_drift_log() {
    tx_estring_value.clear();
    tx_estring_value.append("DRIFT_BEGIN");
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
    delay(10);

    for (int i = 0; i < drift_log_idx; i++) {
        tx_estring_value.clear();
        tx_estring_value.append("DRIFT,");
        tx_estring_value.append((int)drift_log_t[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(drift_log_state[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(drift_log_raw_tof[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(drift_log_yaw[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(drift_log_target_yaw[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(drift_log_yaw_err[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(drift_log_left_pwm[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(drift_log_right_pwm[i]);

        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        BLE.poll();
        delay(3);
    }

    tx_estring_value.clear();
    tx_estring_value.append("DRIFT_END,");
    tx_estring_value.append(drift_log_idx);
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
    delay(10);
}

void reset_map_log() {
    map_log_idx = 0;
}

void log_map_sample(int sample_idx,
                    unsigned long t_ms,
                    float yaw_deg,
                    float tof_mm,
                    float setpoint_deg) {
    if (map_log_idx >= MAX_MAP_LOG) return;

    map_log_t[map_log_idx] = t_ms;
    map_log_step_idx[map_log_idx] = sample_idx;
    map_log_yaw_deg[map_log_idx] = yaw_deg;
    map_log_dist_mm[map_log_idx] = tof_mm;
    map_log_setpoint_deg[map_log_idx] = setpoint_deg;

    map_log_idx++;
}

void send_map_log() {
    tx_estring_value.clear();
    tx_estring_value.append("MAP_BEGIN");
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
    delay(10);

    for (int i = 0; i < map_log_idx; i++) {
        tx_estring_value.clear();
        tx_estring_value.append("MAP,");
        tx_estring_value.append(map_log_step_idx[i]);
        tx_estring_value.append(",");
        tx_estring_value.append((int)map_log_t[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(map_log_yaw_deg[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(map_log_dist_mm[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(map_log_setpoint_deg[i]);

        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        BLE.poll();
        delay(3);
    }

    tx_estring_value.clear();
    tx_estring_value.append("MAP_END,");
    tx_estring_value.append(map_log_idx);
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
    delay(10);
}

void reset_map_turn_controller() {
    reset_orientation_controller_terms();
    last_orientation_ms = millis();
}

void set_map_target_for_step(int sample_idx) {
    map_scan_target_yaw_deg =
        wrap_angle_deg(map_scan_start_yaw_deg - sample_idx * map_scan_step_deg);
    reset_map_turn_controller();
}

void reset_nav_log() {
    nav_log_idx = 0;
}

void log_nav_sample(int mode,
                    unsigned long t_ms,
                    float yaw_deg,
                    float target_yaw_deg,
                    float yaw_err_deg,
                    float tof_mm,
                    int left_pwm,
                    int right_pwm) {
    if (nav_log_idx >= MAX_NAV_LOG) return;

    nav_log_t[nav_log_idx] = t_ms;
    nav_log_mode[nav_log_idx] = mode;
    nav_log_yaw[nav_log_idx] = yaw_deg;
    nav_log_target[nav_log_idx] = target_yaw_deg;
    nav_log_err[nav_log_idx] = yaw_err_deg;
    nav_log_tof[nav_log_idx] = tof_mm;
    nav_log_left_pwm[nav_log_idx] = left_pwm;
    nav_log_right_pwm[nav_log_idx] = right_pwm;

    nav_log_idx++;
}

void send_nav_log() {
    tx_estring_value.clear();
    tx_estring_value.append("NAV_BEGIN");
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
    delay(10);

    for (int i = 0; i < nav_log_idx; i++) {
        tx_estring_value.clear();
        tx_estring_value.append("NAV,");
        tx_estring_value.append((int)nav_log_t[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(nav_log_mode[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(nav_log_yaw[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(nav_log_target[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(nav_log_err[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(nav_log_tof[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(nav_log_left_pwm[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(nav_log_right_pwm[i]);

        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        BLE.poll();
        delay(3);
    }

    tx_estring_value.clear();
    tx_estring_value.append("NAV_END,");
    tx_estring_value.append(nav_log_idx);
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
    delay(10);
}

void disable_all_motion_modes_for_nav() {
    flag_pid_active = false;
    flag_log_active = false;
    flag_pid_rate_test = false;
    flag_orientation_pid = false;
    flag_orientation_log = false;
    flag_step_test = false;
    flag_kf_pid = false;
    flag_drift = false;
    flag_map_scan = false;
    flag_nav_turn = false;
    flag_nav_drive = false;

    drift_state = DRIFT_IDLE;
    map_scan_state = MAP_SCAN_IDLE;
    stop_motors();
}

bool wait_for_nav_yaw(unsigned long timeout_ms) {
    unsigned long start_ms = millis();

    while ((millis() - start_ms) <= timeout_ms) {
        if (update_yaw_from_dmp() && yaw_valid) {
            return true;
        }
        if (yaw_valid) {
            return true;
        }
        BLE.poll();
        delay(2);
    }

    return yaw_valid;
}

void start_nav_turn(float delta_deg,
                    unsigned long timeout_ms,
                    float kp_in,
                    float ki_in,
                    float kd_in) {
    disable_all_motion_modes_for_nav();
    reset_nav_log();

    kp_ang = kp_in;
    ki_ang = ki_in;
    kd_ang = kd_in;

    if (!wait_for_nav_yaw(250)) {
        tx_estring_value.clear();
        tx_estring_value.append("TURN_FAILED,YAW_INVALID");
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        return;
    }

    nav_start_yaw_deg = current_yaw_deg;
    nav_turn_delta_deg = delta_deg;
    nav_target_yaw_deg = wrap_angle_deg(nav_start_yaw_deg + delta_deg);
    nav_timeout_ms = timeout_ms;
    nav_start_ms = millis();
    nav_turn_stable_since_ms = 0;

    reset_orientation_controller_terms();
    last_orientation_ms = nav_start_ms;
    flag_nav_turn = true;

    tx_estring_value.clear();
    tx_estring_value.append("TURN_STARTED,");
    tx_estring_value.append(delta_deg);
    tx_estring_value.append(",");
    tx_estring_value.append(nav_start_yaw_deg);
    tx_estring_value.append(",");
    tx_estring_value.append(nav_target_yaw_deg);
    tx_estring_value.append(",");
    tx_estring_value.append((int)timeout_ms);
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
}

void start_nav_drive(float distance_mm,
                     int base_pwm,
                     unsigned long duration_ms,
                     float heading_kp,
                     int front_stop_mm) {
    disable_all_motion_modes_for_nav();
    reset_nav_log();

    if (duration_ms == 0 || base_pwm <= 0) {
        tx_estring_value.clear();
        tx_estring_value.append("DRIVE_FAILED,BAD_ARGS");
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        return;
    }

    if (!wait_for_nav_yaw(250)) {
        tx_estring_value.clear();
        tx_estring_value.append("DRIVE_FAILED,YAW_INVALID");
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        return;
    }

    nav_drive_direction = (distance_mm >= 0.0f) ? 1 : -1;
    nav_drive_heading_deg = current_yaw_deg;
    nav_drive_base_pwm = clamp_pwm_base(base_pwm);
    nav_drive_duration_ms = (int)duration_ms;
    nav_heading_kp = heading_kp;
    nav_front_stop_mm = front_stop_mm;
    nav_last_tof_mm = -1.0f;
    nav_start_ms = millis();
    flag_nav_drive = true;

    tx_estring_value.clear();
    tx_estring_value.append("DRIVE_STARTED,");
    tx_estring_value.append(distance_mm);
    tx_estring_value.append(",");
    tx_estring_value.append(nav_drive_base_pwm);
    tx_estring_value.append(",");
    tx_estring_value.append(nav_drive_duration_ms);
    tx_estring_value.append(",");
    tx_estring_value.append(nav_drive_heading_deg);
    tx_estring_value.append(",");
    tx_estring_value.append(nav_front_stop_mm);
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
}

void
handle_command()
{   
    // Set the command string from the characteristic value
    robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                             rx_characteristic_string.valueLength());

    bool success;
    int cmd_type = -1;

    // Get robot command type (an integer)
    /* NOTE: THIS SHOULD ALWAYS BE CALLED BEFORE get_next_value()
     * since it uses strtok internally (refer RobotCommand.h and 
     * https://www.cplusplus.com/reference/cstring/strtok/)
     */
    success = robot_cmd.get_command_type(cmd_type);

    // Check if the last tokenization was successful and return if failed
    if (!success) {
        return;
    }

    // Handle the command type accordingly
    switch (cmd_type) {
        /*
         * Write "PONG" on the GATT characteristic BLE_UUID_TX_STRING
         */
        case PING:
            tx_estring_value.clear();
            tx_estring_value.append("PONG");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.print("Sent back: ");
            Serial.println(tx_estring_value.c_str());

            break;
        /*
         * Extract two integers from the command string
         */
        case SEND_TWO_INTS:
            int int_a, int_b;

            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(int_a);
            if (!success)
                return;

            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(int_b);
            if (!success)
                return;

            Serial.print("Two Integers: ");
            Serial.print(int_a);
            Serial.print(", ");
            Serial.println(int_b);
            
            break;
        /*
         * Extract three floats from the command string
         */
        case SEND_THREE_FLOATS:
            float f1,f2,f3;

            success = robot_cmd.get_next_value(f1);
            if (!success)
                return;

            success = robot_cmd.get_next_value(f2);
            if (!success)
                return;

            success = robot_cmd.get_next_value(f3);
            if (!success)
                return;

            Serial.print("Three floats: ");
            Serial.print(f1);
            Serial.print(", ");
            Serial.print(f2);
            Serial.print(", ");
            Serial.println(f3);

            break;
        /*
         * Add a prefix and postfix to the string value extracted from the command string
         */
        case ECHO:

            char char_arr[MAX_MSG_SIZE];

            // Extract the next value from the command string as a character array
            success = robot_cmd.get_next_value(char_arr);
            if (!success)
                return;

            tx_estring_value.clear();
            tx_estring_value.append("Robot says -> ");
            tx_estring_value.append(char_arr);
            tx_estring_value.append(":)");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.println(tx_estring_value.c_str());
            
            break;
        /*
         * DANCE
         */
        case DANCE:
            Serial.println("Look Ma, I'm Dancin'!");

            break;
        
        /*
         * SET_VEL
         */
        case SET_VEL:

            break;

        case GET_TIME_MILLIS:{

            unsigned long t = millis();

            tx_estring_value.clear();
            tx_estring_value.append("T: ");
            tx_estring_value.append(float(t));
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.println(tx_estring_value.c_str());

            break;
        }

        case TIME_LOOP:{

            unsigned long start = millis();

            while(millis() - start <= 3000){
                unsigned long t = millis();
                tx_estring_value.clear();
                tx_estring_value.append("T:");
                tx_estring_value.append(float(t));
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }

            break;
        }

        case SEND_TIME_DATA:{
            
            unsigned long start = millis();

            while(millis() - start <= 3000 && timeIndex < 2000){
                unsigned long t = millis();
                timeArray[timeIndex] = t;
                timeIndex++;
            }

            for (int k=0; k<timeIndex; k++){
                tx_estring_value.clear();
                tx_estring_value.append("T:");
                tx_estring_value.append(int(timeArray[k]));
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }

            if (timeIndex >= 2000){
                Serial.println("Array full");
            }

            break;
        }

        case GET_TEMP_READINGS:{
            unsigned long start = millis();

            while(millis() - start <= 3000 && timeIndex < 2000){
                unsigned long t = millis();
                float temp = getTempDegC();
                timeArray[timeIndex] = t;
                tempArray[timeIndex] = temp;
                timeIndex++;
            }

            for (int k=0; k<timeIndex; k++){
                tx_estring_value.clear();
                tx_estring_value.append("T:");
                tx_estring_value.append(int(timeArray[k]));
                tx_estring_value.append("   ");
                tx_estring_value.append("Temp:");
                tx_estring_value.append(float(tempArray[k]));
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }

            if (timeIndex >= 2000){
                Serial.println("Array full");
            }

            break;
        }

        case READ_IMU_TOF: {
          tof_idx = 0;
          imu_idx = 0;
        
          unsigned long start = millis();
          unsigned long prev_ms = start;
        
          float pitch_cf = 0.0f;
          float roll_cf  = 0.0f;
          bool  cf_init  = false;

          tx_estring_value.clear();
          tx_estring_value.append("DATA_IMU_TOF_STARTED");
          tx_characteristic_string.writeValue(tx_estring_value.c_str());

          while ((millis() - start) <= 3000) {
        
            unsigned long now = millis();

            if (imu_idx < MAX_IMU && myICM.dataReady()) {
              myICM.getAGMT();

              float ax = myICM.accX();
              float ay = myICM.accY();
              float az = myICM.accZ();
        
              float pitch_acc = atan2(ax, az) * 180.0f / M_PI;
              float roll_acc  = atan2(ay, az) * 180.0f / M_PI;

              float dt = (now - prev_ms) / 1000.0f;
              if (dt <= 0) dt = 0.001f;
              prev_ms = now;

              float gx = myICM.gyrX();
              float gy = myICM.gyrY();

              if (!cf_init) {
                pitch_cf = pitch_acc;
                roll_cf  = roll_acc;
                cf_init  = true;
              }
        
              const float a = 0.02f;
              pitch_cf = (pitch_cf + gx * dt) * (1.0f - a) + pitch_acc * a;
              roll_cf  = (roll_cf  + gy * dt) * (1.0f - a) + roll_acc  * a;
        
              imu_t[imu_idx] = now - start;
              pitch_cf_arr[imu_idx] = pitch_cf;
              roll_cf_arr[imu_idx]  = roll_cf;
              imu_idx++;
            }

            // if (tof_idx < MAX_TOF && front.checkForDataReady() && right.checkForDataReady()) {
            //   int dF = front.getDistance();
            //   int dR = right.getDistance();
        
            //   tof_t[tof_idx] = now - start;
            //   tof_front[tof_idx] = dF;
            //   tof_right[tof_idx] = dR;
            //   tof_idx++;
        
            //   front.clearInterrupt();
            //   right.clearInterrupt();
            // }

            if (tof_idx < MAX_TOF && front.checkForDataReady()) {
              int dF = front.getDistance();
            
              tof_t[tof_idx] = now - start;
              tof_front[tof_idx] = dF;
              tof_right[tof_idx] = -1;   // 暂时没有右侧 ToF，先填 -1
              tof_idx++;
            
              front.clearInterrupt();
              // right.clearInterrupt();
            }
        
            BLE.poll();
          }

          tx_estring_value.clear();
          tx_estring_value.append("TOF,t_ms,front_mm,right_mm");
          tx_characteristic_string.writeValue(tx_estring_value.c_str());
          delay(10);
        
          for (int i = 0; i < tof_idx; i++) {
            tx_estring_value.clear();
            tx_estring_value.append((int)tof_t[i]); tx_estring_value.append(",");
            tx_estring_value.append(tof_front[i]);  tx_estring_value.append(",");
            tx_estring_value.append(tof_right[i]);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            BLE.poll();
            delay(2);
          }

          tx_estring_value.clear();
          tx_estring_value.append("IMU,t_ms,pitch,roll");
          tx_characteristic_string.writeValue(tx_estring_value.c_str());
          delay(10);
        
          for (int i = 0; i < imu_idx; i++) {
            tx_estring_value.clear();
            tx_estring_value.append((int)imu_t[i]); tx_estring_value.append(",");
            tx_estring_value.append(pitch_cf_arr[i]); tx_estring_value.append(",");
            tx_estring_value.append(roll_cf_arr[i]);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            BLE.poll();
            delay(2);
          }

          tx_estring_value.clear();
          tx_estring_value.append("DONE,TOF_N=");
          tx_estring_value.append(tof_idx);
          tx_estring_value.append(",IMU_N=");
          tx_estring_value.append(imu_idx);
          tx_characteristic_string.writeValue(tx_estring_value.c_str());
        
          break;
        }

        case SET_PID_GAINS: {
            success = robot_cmd.get_next_value(kp);
            if (!success) return;
        
            success = robot_cmd.get_next_value(ki);
            if (!success) return;
        
            success = robot_cmd.get_next_value(kd);
            if (!success) return;
        
            tx_estring_value.clear();
            tx_estring_value.append("PID_GAINS,");
            tx_estring_value.append(kp);
            tx_estring_value.append(",");
            tx_estring_value.append(ki);
            tx_estring_value.append(",");
            tx_estring_value.append(kd);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
        
            Serial.print("PID gains set: ");
            Serial.print(kp);
            Serial.print(", ");
            Serial.print(ki);
            Serial.print(", ");
            Serial.println(kd);
        
            break;
        }
        
        case START_PID_LOG: {
            reset_pid_log();
            flag_orientation_pid = false;
            flag_orientation_log = false;
            flag_step_test = false;
            flag_pid_rate_test = false;
            flag_kf_pid = false;
            flag_drift = false;
            flag_map_scan = false;
            flag_nav_turn = false;
            flag_nav_drive = false;
            drift_state = DRIFT_IDLE;
            map_scan_state = MAP_SCAN_IDLE;
            stop_motors();
        
            pid_start_ms = millis();
            last_pid_ms = pid_start_ms;
        
            flag_pid_active = true;
            flag_log_active = true;
        
            tx_estring_value.clear();
            tx_estring_value.append("PID_LOG_STARTED");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
        
            Serial.println("PID logging started");
        
            break;
        }
        
        case STOP_PID_LOG: {
            flag_pid_active = false;
            flag_log_active = false;
            stop_motors();
        
            tx_estring_value.clear();
            tx_estring_value.append("PID_LOG_STOPPED");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
        
            Serial.println("PID logging stopped");
        
            break;
        }
        
        case SEND_PID_LOG: {
            send_pid_log();
        
            Serial.print("Sent PID log, N=");
            Serial.println(log_idx);
        
            break;
        }

        case START_PID_RATE_TEST: {
            pid_rate_idx = 0;
            flag_pid_active = false;
            flag_log_active = false;
            flag_orientation_pid = false;
            flag_orientation_log = false;
            flag_step_test = false;
            flag_kf_pid = false;
            flag_drift = false;
            flag_map_scan = false;
            flag_nav_turn = false;
            flag_nav_drive = false;
            drift_state = DRIFT_IDLE;
            map_scan_state = MAP_SCAN_IDLE;
            stop_motors();
            flag_pid_rate_test = true;
            pid_start_ms = millis();
            last_pid_ms = pid_start_ms;
        
            tx_estring_value.clear();
            tx_estring_value.append("PID_RATE_STARTED");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
        
            Serial.println("PID rate test started");
            break;
        }

        case SEND_PID_RATE_TEST: {
            tx_estring_value.clear();
            tx_estring_value.append("PID_RATE_BEGIN");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            delay(10);
        
            for (int i = 0; i < pid_rate_idx; i++) {
                tx_estring_value.clear();
                tx_estring_value.append("PR,");
                tx_estring_value.append((int)pid_rate_t[i]);
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
                BLE.poll();
                delay(2);
            }
        
            tx_estring_value.clear();
            tx_estring_value.append("PID_RATE_END,");
            tx_estring_value.append(pid_rate_idx);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            delay(10);
        
            Serial.print("Sent PID rate log, N=");
            Serial.println(pid_rate_idx);
            break;
        }

        case PID_ORIENTATION: {
            int duration_ms = 0;

            success = robot_cmd.get_next_value(duration_ms);
            if (!success) return;
        
            success = robot_cmd.get_next_value(setpoint_yaw_deg);
            if (!success) return;
        
            success = robot_cmd.get_next_value(kp_ang);
            if (!success) return;
        
            success = robot_cmd.get_next_value(ki_ang);
            if (!success) return;
        
            success = robot_cmd.get_next_value(kd_ang);
            if (!success) return;

            flag_pid_active = false;
            flag_log_active = false;
            flag_orientation_pid = false;
            flag_orientation_log = false;
            flag_step_test = false;
            flag_pid_rate_test = false;
            flag_kf_pid = false;
            flag_drift = false;
            flag_map_scan = false;
            flag_nav_turn = false;
            flag_nav_drive = false;
            drift_state = DRIFT_IDLE;
            map_scan_state = MAP_SCAN_IDLE;
            stop_motors();

            reset_orientation_log();
        
            orientation_test_duration_ms = (unsigned long)duration_ms;
            setpoint_yaw_deg = wrap_angle_deg(setpoint_yaw_deg);
        
            orientation_start_ms = millis();
            last_orientation_ms = orientation_start_ms;
        
            flag_orientation_pid = true;
            flag_orientation_log = true;
        
            tx_estring_value.clear();
            tx_estring_value.append("ORIENTATION_PID_STARTED,");
            tx_estring_value.append((int)orientation_test_duration_ms);
            tx_estring_value.append(",");
            tx_estring_value.append(setpoint_yaw_deg);
            tx_estring_value.append(",");
            tx_estring_value.append(kp_ang);
            tx_estring_value.append(",");
            tx_estring_value.append(ki_ang);
            tx_estring_value.append(",");
            tx_estring_value.append(kd_ang);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
        
            Serial.println("Orientation PID started");
            break;
        }

        case SET_ANGULAR_SETPOINT: {
            success = robot_cmd.get_next_value(setpoint_yaw_deg);
            if (!success) return;

            setpoint_yaw_deg = wrap_angle_deg(setpoint_yaw_deg);

            tx_estring_value.clear();
            tx_estring_value.append("SETPOINT_UPDATED,");
            tx_estring_value.append(setpoint_yaw_deg);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.print("New angular setpoint: ");
            Serial.println(setpoint_yaw_deg);
            break;
        }

        case STOP_ORIENTATION_PID: {
            flag_pid_active = false;
            flag_log_active = false;
            flag_orientation_pid = false;
            flag_orientation_log = false;
            flag_step_test = false;
            flag_kf_pid = false;
            flag_map_scan = false;
            flag_nav_turn = false;
            flag_nav_drive = false;
            map_scan_state = MAP_SCAN_IDLE;
            stop_motors();

            tx_estring_value.clear();
            tx_estring_value.append("ORIENTATION_PID_STOPPED");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.println("Orientation PID stopped");
            break;
        }

        case SEND_ANGULAR_DATA: {
            send_angular_data();

            Serial.print("Sent angular log, N=");
            Serial.println(ang_idx);
            break;
        }

        case START_STEP_RESPONSE: {
            int pwm_in = 130;
            int stop_mm_in = 1000;

            success = robot_cmd.get_next_value(pwm_in);
            if (!success) return;

            success = robot_cmd.get_next_value(stop_mm_in);
            if (!success) return;

            flag_pid_active = false;
            flag_log_active = false;
            flag_orientation_pid = false;
            flag_orientation_log = false;
            flag_pid_rate_test = false;
            flag_map_scan = false;
            flag_nav_turn = false;
            flag_nav_drive = false;
            map_scan_state = MAP_SCAN_IDLE;
            stop_motors();

            reset_step_log();

            step_base_pwm_cmd = clamp_pwm_base(pwm_in);
            step_stop_dist_mm = stop_mm_in;
            step_start_ms = millis();

            flag_step_test = true;

            drive_forward_pwm(step_base_pwm_cmd);

            tx_estring_value.clear();
            tx_estring_value.append("STEP_STARTED,");
            tx_estring_value.append(step_base_pwm_cmd);
            tx_estring_value.append(",");
            tx_estring_value.append(step_stop_dist_mm);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.print("Step response started. pwm=");
            Serial.print(step_base_pwm_cmd);
            Serial.print(", stop_mm=");
            Serial.println(step_stop_dist_mm);
            break;
        }

        case STOP_STEP_RESPONSE: {
            flag_step_test = false;
            stop_motors();

            tx_estring_value.clear();
            tx_estring_value.append("STEP_STOPPED");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.println("Step response manually stopped");
            break;
        }

        case SEND_STEP_RESPONSE: {
            send_step_log();

            Serial.print("Sent step log, N=");
            Serial.println(step_idx);
            break;
        }

        case START_KF_PID_LOG: {
            flag_pid_active = false;
            flag_log_active = false;
            flag_orientation_pid = false;
            flag_orientation_log = false;
            flag_step_test = false;
            flag_pid_rate_test = false;
            flag_map_scan = false;
            flag_nav_turn = false;
            flag_nav_drive = false;
            map_scan_state = MAP_SCAN_IDLE;
            stop_motors();

            reset_pid_log();
            reset_kf_log();

            kf_initialized = false;
            kf_last_u_scaled = 0.0f;
            kf_est_dist_mm = 0.0f;
            kf_est_vel_mm_s = 0.0f;

            pid_start_ms = millis();
            last_pid_ms = pid_start_ms;
            flag_kf_pid = true;

            tx_estring_value.clear();
            tx_estring_value.append("KF_PID_LOG_STARTED");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.println("KF PID mode started");
            break;
        }

        case STOP_KF_PID_LOG: {
            flag_kf_pid = false;
            stop_motors();

            tx_estring_value.clear();
            tx_estring_value.append("KF_PID_LOG_STOPPED");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.println("KF PID mode stopped");
            break;
        }

        case SEND_KF_LOG: {
            send_kf_log();

            Serial.print("Sent KF log, N=");
            Serial.println(kf_log_idx);
            break;
        }

        case START_DRIFT: {
            int pwm_in = 150;
            int trigger_mm_in = 950;
            float kp_in = 0.8f;
            float ki_in = 0.001f;
            float kd_in = 0.5f;
            int exit_pwm_in = 150;
            int exit_ms_in = 400;
        
            success = robot_cmd.get_next_value(pwm_in);
            if (!success) return;
        
            success = robot_cmd.get_next_value(trigger_mm_in);
            if (!success) return;
        
            success = robot_cmd.get_next_value(kp_in);
            if (!success) return;
        
            success = robot_cmd.get_next_value(ki_in);
            if (!success) return;
        
            success = robot_cmd.get_next_value(kd_in);
            if (!success) return;
        
            success = robot_cmd.get_next_value(exit_pwm_in);
            if (!success) return;
        
            success = robot_cmd.get_next_value(exit_ms_in);
            if (!success) return;
        
            flag_pid_active = false;
            flag_log_active = false;
            flag_orientation_pid = false;
            flag_orientation_log = false;
            flag_step_test = false;
            flag_pid_rate_test = false;
            flag_kf_pid = false;
            flag_map_scan = false;
            flag_nav_turn = false;
            flag_nav_drive = false;
            map_scan_state = MAP_SCAN_IDLE;
            stop_motors();
        
            reset_drift_log();
        
            drift_forward_pwm = clamp_pwm_base(pwm_in);
            drift_trigger_mm = trigger_mm_in;
        
            // Keep receiving BLE gain parameters for notebook compatibility,
            // but the drift turn itself now uses a fixed snap-turn profile.
            kp_ang = kp_in;
            ki_ang = ki_in;
            kd_ang = kd_in;
        
            drift_exit_pwm = clamp_pwm_base(exit_pwm_in);
            drift_exit_duration_ms = (unsigned long)exit_ms_in;
            drift_turn_pwm_cmd = clamp_pwm_turn(DRIFT_TURN_PWM_FIXED);
            drift_turn_release_progress_deg = 180.0f - DRIFT_TURN_LEAD_DEG;
        
            drift_state = DRIFT_FORWARD;
            flag_drift = true;
            drift_start_ms = millis();
            drift_turn_start_ms = 0;
            drift_exit_start_ms = 0;
            drift_has_target = false;
        
            integral_yaw_error = 0.0f;
            prev_yaw_error_deg = 0.0f;
            p_term_ang = 0.0f;
            i_term_ang = 0.0f;
            d_term_ang = 0.0f;
            d_term_ang_filtered = 0.0f;
            last_d_term_ang_filtered = 0.0f;
            yaw_error_deg = 0.0f;
            drift_start_yaw_deg = 0.0f;
            drift_last_yaw_deg = 0.0f;
            drift_turn_progress_deg = 0.0f;
            drift_turn_dir = 0;
        
            drive_forward_pwm(drift_forward_pwm);
        
            tx_estring_value.clear();
            tx_estring_value.append("DRIFT_STARTED,");
            tx_estring_value.append(drift_forward_pwm);
            tx_estring_value.append(",");
            tx_estring_value.append(drift_trigger_mm);
            tx_estring_value.append(",");
            tx_estring_value.append(kp_ang);
            tx_estring_value.append(",");
            tx_estring_value.append(ki_ang);
            tx_estring_value.append(",");
            tx_estring_value.append(kd_ang);
            tx_estring_value.append(",");
            tx_estring_value.append(drift_exit_pwm);
            tx_estring_value.append(",");
            tx_estring_value.append((int)drift_exit_duration_ms);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
        
            Serial.print("Drift started. pwm=");
            Serial.print(drift_forward_pwm);
            Serial.print(", trigger_mm=");
            Serial.print(drift_trigger_mm);
            Serial.print(", kp=");
            Serial.print(kp_ang);
            Serial.print(", ki=");
            Serial.print(ki_ang);
            Serial.print(", kd=");
            Serial.print(kd_ang);
            Serial.print(", exit_pwm=");
            Serial.print(drift_exit_pwm);
            Serial.print(", exit_ms=");
            Serial.println((int)drift_exit_duration_ms);
            break;
        }

        case STOP_DRIFT: {
            flag_drift = false;
            drift_state = DRIFT_IDLE;
            stop_motors();
        
            tx_estring_value.clear();
            tx_estring_value.append("DRIFT_STOPPED");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
        
            Serial.println("Drift stopped");
            break;
        }

        case SEND_DRIFT_LOG: {
            send_drift_log();
        
            Serial.print("Sent drift log, N=");
            Serial.println(drift_log_idx);
            break;
        }

        case START_MAP_SCAN: {
            float step_deg_in = 20.0f;
            int num_steps_in = 18;
            float kp_in = 0.8f;
            float ki_in = 0.001f;
            float kd_in = 0.5f;
            int settle_ms_in = 150;

            success = robot_cmd.get_next_value(step_deg_in);
            if (!success) return;

            success = robot_cmd.get_next_value(num_steps_in);
            if (!success) return;

            success = robot_cmd.get_next_value(kp_in);
            if (!success) return;

            success = robot_cmd.get_next_value(ki_in);
            if (!success) return;

            success = robot_cmd.get_next_value(kd_in);
            if (!success) return;

            success = robot_cmd.get_next_value(settle_ms_in);
            if (!success) return;

            if (step_deg_in <= 0.0f || num_steps_in <= 0) {
                tx_estring_value.clear();
                tx_estring_value.append("MAP_SCAN_BAD_ARGS");
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
                return;
            }

            flag_pid_active = false;
            flag_log_active = false;
            flag_orientation_pid = false;
            flag_orientation_log = false;
            flag_step_test = false;
            flag_pid_rate_test = false;
            flag_kf_pid = false;
            flag_drift = false;
            flag_map_scan = false;
            drift_state = DRIFT_IDLE;
            map_scan_state = MAP_SCAN_IDLE;
            stop_motors();

            reset_map_log();
            reset_orientation_controller_terms();

            kp_ang = kp_in;
            ki_ang = ki_in;
            kd_ang = kd_in;

            map_scan_step_deg = step_deg_in;
            map_scan_num_steps = num_steps_in;
            if (map_scan_num_steps > MAX_MAP_LOG) {
                map_scan_num_steps = MAX_MAP_LOG;
            }
            map_scan_settle_ms =
                (unsigned long)((settle_ms_in > 0) ? settle_ms_in : 0);

            bool got_start_yaw = false;
            unsigned long yaw_wait_start_ms = millis();
            while ((millis() - yaw_wait_start_ms) < 250) {
                if (update_yaw_from_dmp() && yaw_valid) {
                    got_start_yaw = true;
                    break;
                }
                BLE.poll();
                delay(2);
            }

            if (!got_start_yaw) {
                tx_estring_value.clear();
                tx_estring_value.append("MAP_SCAN_YAW_INVALID");
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
                Serial.println("Map scan start failed: yaw invalid");
                return;
            }

            map_scan_start_ms = millis();
            map_scan_settle_start_ms = map_scan_start_ms;
            map_scan_start_yaw_deg = current_yaw_deg;
            map_scan_sample_idx = 0;
            set_map_target_for_step(map_scan_sample_idx);
            map_scan_state = MAP_SCAN_SETTLING;
            flag_map_scan = true;
            stop_motors();

            tx_estring_value.clear();
            tx_estring_value.append("MAP_SCAN_STARTED,");
            tx_estring_value.append(map_scan_step_deg);
            tx_estring_value.append(",");
            tx_estring_value.append(map_scan_num_steps);
            tx_estring_value.append(",");
            tx_estring_value.append(kp_ang);
            tx_estring_value.append(",");
            tx_estring_value.append(ki_ang);
            tx_estring_value.append(",");
            tx_estring_value.append(kd_ang);
            tx_estring_value.append(",");
            tx_estring_value.append((int)map_scan_settle_ms);
            tx_estring_value.append(",");
            tx_estring_value.append(map_scan_start_yaw_deg);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.print("Map scan started. step_deg=");
            Serial.print(map_scan_step_deg);
            Serial.print(", num_steps=");
            Serial.print(map_scan_num_steps);
            Serial.print(", settle_ms=");
            Serial.println((int)map_scan_settle_ms);
            break;
        }

        case STOP_MAP_SCAN: {
            flag_map_scan = false;
            map_scan_state = MAP_SCAN_IDLE;
            stop_motors();

            tx_estring_value.clear();
            tx_estring_value.append("MAP_SCAN_STOPPED");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.println("Map scan stopped");
            break;
        }

        case SEND_MAP_SCAN: {
            send_map_log();

            Serial.print("Sent map log, N=");
            Serial.println(map_log_idx);
            break;
        }

        case TURN_REL_DEG: {
            float delta_deg = 0.0f;
            int timeout_ms_in = 3000;
            float kp_in = 0.8f;
            float ki_in = 0.001f;
            float kd_in = 0.2f;

            success = robot_cmd.get_next_value(delta_deg);
            if (!success) return;

            success = robot_cmd.get_next_value(timeout_ms_in);
            if (!success) return;

            success = robot_cmd.get_next_value(kp_in);
            if (!success) return;

            success = robot_cmd.get_next_value(ki_in);
            if (!success) return;

            success = robot_cmd.get_next_value(kd_in);
            if (!success) return;

            if (timeout_ms_in <= 0) {
                tx_estring_value.clear();
                tx_estring_value.append("TURN_FAILED,BAD_ARGS");
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
                return;
            }

            start_nav_turn(delta_deg,
                           (unsigned long)timeout_ms_in,
                           kp_in,
                           ki_in,
                           kd_in);
            break;
        }

        case DRIVE_CELL_MM: {
            float distance_mm = 304.8f;
            int base_pwm = 90;
            int duration_ms_in = 900;
            float heading_kp = 1.2f;
            int front_stop_mm = 250;

            success = robot_cmd.get_next_value(distance_mm);
            if (!success) return;

            success = robot_cmd.get_next_value(base_pwm);
            if (!success) return;

            success = robot_cmd.get_next_value(duration_ms_in);
            if (!success) return;

            success = robot_cmd.get_next_value(heading_kp);
            if (!success) return;

            success = robot_cmd.get_next_value(front_stop_mm);
            if (!success) return;

            if (duration_ms_in <= 0) {
                tx_estring_value.clear();
                tx_estring_value.append("DRIVE_FAILED,BAD_ARGS");
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
                return;
            }

            start_nav_drive(distance_mm,
                            base_pwm,
                            (unsigned long)duration_ms_in,
                            heading_kp,
                            front_stop_mm);
            break;
        }

        case STOP_NAV: {
            disable_all_motion_modes_for_nav();

            tx_estring_value.clear();
            tx_estring_value.append("NAV_STOPPED");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            break;
        }

        case SEND_NAV_LOG: {
            send_nav_log();

            Serial.print("Sent nav log, N=");
            Serial.println(nav_log_idx);
            break;
        }
        
        /* 
         * The default case may not capture all types of invalid commands.
         * It is safer to validate the command string on the central device (in python)
         * before writing to the characteristic.
         */
        default:
            Serial.print("Invalid Command Type: ");
            Serial.println(cmd_type);
            break;
    }
}

void
setup()
{
    Serial.begin(115200);

    pinMode(right_IN1, OUTPUT);
    pinMode(right_IN2, OUTPUT);
    pinMode(left_IN3, OUTPUT);
    pinMode(left_IN4, OUTPUT);
    stop_motors();

    /// Lab3 ///
    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);
    
    myICM.begin(WIRE_PORT, AD0_VAL);

    if (myICM.status == ICM_20948_Stat_Ok) {
        setup_dmp();
    }
    
    /// Lab3 ///

    BLE.begin();

    /// Lab3 ///
    // pinMode(XSHUT, OUTPUT);
    // digitalWrite(XSHUT, LOW);
    // delay(10);
    
    // right.setI2CAddress(0x30);
    
    // if (right.begin() != 0) {
    //   Serial.println("Right ToF failed");
    // }
    
    // digitalWrite(XSHUT, HIGH);
    // delay(10);
    
    if (front.begin() != 0) {
      Serial.println("Front ToF failed");
    } else {
      front.setDistanceModeLong();
      front.setTimingBudgetInMs(33);      
      front.setIntermeasurementPeriod(40); 
      front.startRanging();
    }
    
    // right.startRanging();
    /// Lab3 ///

    // Set advertised local name and service
    BLE.setDeviceName("Artemis BLE");
    BLE.setLocalName("Artemis BLE");
    BLE.setAdvertisedService(testService);

    // Add BLE characteristics
    testService.addCharacteristic(tx_characteristic_float);
    testService.addCharacteristic(tx_characteristic_string);
    testService.addCharacteristic(rx_characteristic_string);

    // Add BLE service
    BLE.addService(testService);

    // Initial values for characteristics
    // Set initial values to prevent errors when reading for the first time on central devices
    tx_characteristic_float.writeValue(0.0);

    /*
     * An example using the EString
     */
    // Clear the contents of the EString before using it
    tx_estring_value.clear();

    // Append the string literal "[->"
    tx_estring_value.append("[->");

    // Append the float value
    tx_estring_value.append(9.0);

    // Append the string literal "<-]"
    tx_estring_value.append("<-]");

    // Write the value to the characteristic
    tx_characteristic_string.writeValue(tx_estring_value.c_str());

    // Output MAC Address
    Serial.print("Advertising BLE with MAC: ");
    Serial.println(BLE.address());

    BLE.advertise();
}

void
write_data()
{
    currentMillis = millis();
    if (currentMillis - previousMillis > interval) {

        tx_float_value = tx_float_value + 0.5;
        tx_characteristic_float.writeValue(tx_float_value);

        if (tx_float_value > 10000) {
            tx_float_value = 0;
            
        }

        previousMillis = currentMillis;
    }
}

void
read_data()
{
    // Query if the characteristic value has been written by another BLE device
    if (rx_characteristic_string.written()) {
        handle_command();
    }
}

void
loop()
{
    BLEDevice central = BLE.central();

    if (central) {
        Serial.print("Connected to: ");
        Serial.println(central.address());

        while (central.connected()) {

            // 先处理电脑发来的 BLE 命令
            read_data();

            if (flag_drift) {
                unsigned long now = millis();
            
                if ((now - drift_start_ms) >= drift_max_runtime_ms) {
                    flag_drift = false;
                    drift_state = DRIFT_DONE;
                    stop_motors();
            
                    tx_estring_value.clear();
                    tx_estring_value.append("DRIFT_TIMEOUT");
                    tx_characteristic_string.writeValue(tx_estring_value.c_str());
            
                    Serial.println("Drift timeout");
                }
            
                bool got_new_tof = false;
            
                if (front.checkForDataReady()) {
                    current_dist_mm = front.getDistance();
                    front.clearInterrupt();
                    got_new_tof = true;
                }

                bool yaw_ok = update_yaw_from_dmp();
                bool yaw_ready = yaw_ok || yaw_valid;
            
                if (drift_state == DRIFT_FORWARD) {
                    drive_forward_pwm(drift_forward_pwm);
                
                    if (got_new_tof && current_dist_mm <= drift_trigger_mm) {
                        stop_motors();
                
                        if (yaw_ready) {
                            drift_start_yaw_deg = current_yaw_deg;
                            drift_last_yaw_deg = current_yaw_deg;
                            drift_turn_progress_deg = 0.0f;

                            float proposed_target_yaw_deg = wrap_angle_deg(current_yaw_deg + 180.0f);
                            float initial_turn_error_deg =
                                wrap_angle_deg(proposed_target_yaw_deg - current_yaw_deg);

                            // Keep the turn-progress sign aligned with Lab 6:
                            // positive yaw error turns right, negative yaw error turns left.
                            drift_turn_dir = (initial_turn_error_deg < 0.0f) ? 1 : -1;
                            drift_target_yaw_deg =
                                wrap_angle_deg(drift_start_yaw_deg + 180.0f * drift_turn_dir);
                            drift_has_target = true;
                            drift_state = DRIFT_TURN;
                            drift_turn_start_ms = now;
                
                            integral_yaw_error = 0.0f;
                            prev_yaw_error_deg = 0.0f;
                            d_term_ang_filtered = 0.0f;
                            last_d_term_ang_filtered = 0.0f;
                            last_orientation_ms = now;
                
                            tx_estring_value.clear();
                            tx_estring_value.append("DRIFT_TURN_START,");
                            tx_estring_value.append(current_dist_mm);
                            tx_estring_value.append(",");
                            tx_estring_value.append(drift_target_yaw_deg);
                            tx_estring_value.append(",");
                            tx_estring_value.append(drift_turn_pwm_cmd);
                            tx_estring_value.append(",");
                            tx_estring_value.append(drift_turn_release_progress_deg);
                            tx_characteristic_string.writeValue(tx_estring_value.c_str());

                            if (drift_turn_dir > 0) {
                                turn_right_pwm(drift_turn_pwm_cmd);
                            } else {
                                turn_left_pwm(drift_turn_pwm_cmd);
                            }
                        } else {
                            flag_drift = false;
                            drift_state = DRIFT_DONE;
                            stop_motors();
                
                            tx_estring_value.clear();
                            tx_estring_value.append("DRIFT_YAW_INVALID");
                            tx_characteristic_string.writeValue(tx_estring_value.c_str());
                        }
                    }
                }

                else if (drift_state == DRIFT_TURN) {
                    if (yaw_ok && drift_has_target) {
                        last_orientation_ms = now;

                        float delta_yaw_deg =
                            wrap_angle_deg(current_yaw_deg - drift_last_yaw_deg);
                        drift_turn_progress_deg += delta_yaw_deg;
                        drift_last_yaw_deg = current_yaw_deg;

                        float target_progress_deg = 180.0f * drift_turn_dir;
                        yaw_error_deg = target_progress_deg - drift_turn_progress_deg;

                        float signed_turn_progress_deg =
                            drift_turn_progress_deg * drift_turn_dir;

                        if (signed_turn_progress_deg >= drift_turn_release_progress_deg) {
                            drift_state = DRIFT_EXIT;
                            drift_exit_start_ms = now;

                            drive_forward_pwm(drift_exit_pwm);

                            tx_estring_value.clear();
                            tx_estring_value.append("DRIFT_EXIT_START,");
                            tx_estring_value.append(drift_exit_pwm);
                            tx_estring_value.append(",");
                            tx_estring_value.append((int)drift_exit_duration_ms);
                            tx_estring_value.append(",");
                            tx_estring_value.append(signed_turn_progress_deg);
                            tx_estring_value.append(",");
                            tx_estring_value.append(current_yaw_deg);
                            tx_characteristic_string.writeValue(tx_estring_value.c_str());
                        } else {
                            if (drift_turn_dir > 0) {
                                turn_right_pwm(drift_turn_pwm_cmd);
                            } else {
                                turn_left_pwm(drift_turn_pwm_cmd);
                            }
                        }
                
                        prev_yaw_error_deg = yaw_error_deg;
                    }
                }

                else if (drift_state == DRIFT_EXIT) {
                    drive_forward_pwm(drift_exit_pwm);
                
                    if ((now - drift_exit_start_ms) >= drift_exit_duration_ms) {
                        stop_motors();
                        flag_drift = false;
                        drift_state = DRIFT_DONE;
                
                        tx_estring_value.clear();
                        tx_estring_value.append("DRIFT_DONE");
                        tx_characteristic_string.writeValue(tx_estring_value.c_str());
                
                        Serial.println("Drift done");
                    }
                }

                float drift_target_yaw_for_log = drift_has_target ? drift_target_yaw_deg : 0.0f;
                float drift_yaw_err_for_log = drift_has_target ? yaw_error_deg : 0.0f;
                
                log_drift_sample(now - drift_start_ms,
                                 (int)drift_state,
                                 current_dist_mm,
                                 current_yaw_deg,
                                 drift_target_yaw_for_log,
                                 drift_yaw_err_for_log,
                                 actual_left_pwm,
                                 actual_right_pwm);
            }

            if (flag_map_scan) {
                unsigned long now = millis();
                bool got_new_yaw = update_yaw_from_dmp();

                if (map_scan_state == MAP_SCAN_TURNING) {
                    if (got_new_yaw && yaw_valid) {
                        unsigned long dt_ms = now - last_orientation_ms;
                        last_orientation_ms = now;

                        float dt = dt_ms / 1000.0f;
                        if (dt <= 0.0f) dt = 0.001f;

                        yaw_error_deg =
                            wrap_angle_deg(map_scan_target_yaw_deg - current_yaw_deg);

                        p_term_ang = kp_ang * yaw_error_deg;

                        if (ki_ang == 0.0f) {
                            integral_yaw_error = 0.0f;
                        } else {
                            integral_yaw_error += yaw_error_deg * dt;

                            if (integral_yaw_error > ANG_INTEGRAL_LIMIT) {
                                integral_yaw_error = ANG_INTEGRAL_LIMIT;
                            }
                            if (integral_yaw_error < -ANG_INTEGRAL_LIMIT) {
                                integral_yaw_error = -ANG_INTEGRAL_LIMIT;
                            }
                        }
                        i_term_ang = ki_ang * integral_yaw_error;

                        float derivative_ang = 0.0f;
                        if (yaw_measurement_initialized) {
                            float delta_yaw_deg =
                                wrap_angle_deg(current_yaw_deg - prev_yaw_measurement_deg);
                            derivative_ang = -delta_yaw_deg / dt;
                        } else {
                            yaw_measurement_initialized = true;
                        }
                        prev_yaw_measurement_deg = current_yaw_deg;

                        if (fabs(derivative_ang) > 1000.0f) {
                            derivative_ang = 0.0f;
                        }

                        d_term_ang = kd_ang * derivative_ang;
                        d_term_ang_filtered =
                            D_ANG_LPF_ALPHA * d_term_ang +
                            (1.0f - D_ANG_LPF_ALPHA) * last_d_term_ang_filtered;
                        last_d_term_ang_filtered = d_term_ang_filtered;

                        u_cmd = p_term_ang + i_term_ang + d_term_ang_filtered;

                        int turn_pwm = clamp_pwm_turn((int)fabs(u_cmd));

                        if (fabs(yaw_error_deg) <= MAP_YAW_TOL_DEG) {
                            stop_motors();
                            map_scan_state = MAP_SCAN_SETTLING;
                            map_scan_settle_start_ms = now;
                        } else {
                            if (yaw_error_deg > 0.0f) {
                                turn_right_pwm(turn_pwm);
                            } else {
                                turn_left_pwm(turn_pwm);
                            }
                        }

                        prev_yaw_error_deg = yaw_error_deg;
                    }
                }

                else if (map_scan_state == MAP_SCAN_SETTLING) {
                    stop_motors();

                    if ((now - map_scan_settle_start_ms) >= map_scan_settle_ms) {
                        if (front.checkForDataReady()) {
                            float d_mm = front.getDistance();
                            front.clearInterrupt();

                            log_map_sample(map_scan_sample_idx,
                                           now - map_scan_start_ms,
                                           current_yaw_deg,
                                           d_mm,
                                           map_scan_target_yaw_deg);

                            tx_estring_value.clear();
                            tx_estring_value.append("MAP_SAMPLE,");
                            tx_estring_value.append(map_scan_sample_idx);
                            tx_estring_value.append(",");
                            tx_estring_value.append(current_yaw_deg);
                            tx_estring_value.append(",");
                            tx_estring_value.append(d_mm);
                            tx_estring_value.append(",");
                            tx_estring_value.append(map_scan_target_yaw_deg);
                            tx_characteristic_string.writeValue(tx_estring_value.c_str());

                            if (map_scan_sample_idx >= (map_scan_num_steps - 1) ||
                                map_log_idx >= MAX_MAP_LOG) {
                                flag_map_scan = false;
                                map_scan_state = MAP_SCAN_DONE;
                                stop_motors();

                                tx_estring_value.clear();
                                tx_estring_value.append("MAP_SCAN_DONE,");
                                tx_estring_value.append(map_log_idx);
                                tx_characteristic_string.writeValue(tx_estring_value.c_str());

                                Serial.print("Map scan done. N=");
                                Serial.println(map_log_idx);
                            } else {
                                map_scan_sample_idx++;
                                set_map_target_for_step(map_scan_sample_idx);
                                map_scan_state = MAP_SCAN_TURNING;

                                tx_estring_value.clear();
                                tx_estring_value.append("MAP_STEP_TARGET,");
                                tx_estring_value.append(map_scan_sample_idx);
                                tx_estring_value.append(",");
                                tx_estring_value.append(map_scan_target_yaw_deg);
                                tx_characteristic_string.writeValue(tx_estring_value.c_str());
                            }
                        }
                    }
                }
            }

            if (flag_nav_turn) {
                unsigned long now = millis();
                bool got_new_yaw = update_yaw_from_dmp();

                if ((now - nav_start_ms) >= nav_timeout_ms) {
                    flag_nav_turn = false;
                    stop_motors();

                    float timeout_err = yaw_valid
                        ? wrap_angle_deg(nav_target_yaw_deg - current_yaw_deg)
                        : 0.0f;

                    tx_estring_value.clear();
                    tx_estring_value.append("TURN_FAILED,TIMEOUT,");
                    tx_estring_value.append(current_yaw_deg);
                    tx_estring_value.append(",");
                    tx_estring_value.append(timeout_err);
                    tx_characteristic_string.writeValue(tx_estring_value.c_str());
                } else if (got_new_yaw && yaw_valid) {
                    unsigned long dt_ms = now - last_orientation_ms;
                    last_orientation_ms = now;

                    float dt = dt_ms / 1000.0f;
                    if (dt <= 0.0f) dt = 0.001f;

                    yaw_error_deg = wrap_angle_deg(nav_target_yaw_deg - current_yaw_deg);
                    p_term_ang = kp_ang * yaw_error_deg;

                    if (ki_ang == 0.0f) {
                        integral_yaw_error = 0.0f;
                    } else {
                        integral_yaw_error += yaw_error_deg * dt;

                        if (integral_yaw_error > ANG_INTEGRAL_LIMIT) {
                            integral_yaw_error = ANG_INTEGRAL_LIMIT;
                        }
                        if (integral_yaw_error < -ANG_INTEGRAL_LIMIT) {
                            integral_yaw_error = -ANG_INTEGRAL_LIMIT;
                        }
                    }
                    i_term_ang = ki_ang * integral_yaw_error;

                    float derivative_ang = 0.0f;
                    if (yaw_measurement_initialized) {
                        float delta_yaw_deg =
                            wrap_angle_deg(current_yaw_deg - prev_yaw_measurement_deg);
                        derivative_ang = -delta_yaw_deg / dt;
                    } else {
                        yaw_measurement_initialized = true;
                    }
                    prev_yaw_measurement_deg = current_yaw_deg;

                    if (fabs(derivative_ang) > 1000.0f) {
                        derivative_ang = 0.0f;
                    }

                    d_term_ang = kd_ang * derivative_ang;
                    d_term_ang_filtered =
                        D_ANG_LPF_ALPHA * d_term_ang +
                        (1.0f - D_ANG_LPF_ALPHA) * last_d_term_ang_filtered;
                    last_d_term_ang_filtered = d_term_ang_filtered;

                    u_cmd = p_term_ang + i_term_ang + d_term_ang_filtered;

                    bool turn_in_tolerance = fabs(yaw_error_deg) <= NAV_TURN_TOL_DEG;
                    if (turn_in_tolerance) {
                        if (nav_turn_stable_since_ms == 0) {
                            nav_turn_stable_since_ms = now;
                        }
                    } else {
                        nav_turn_stable_since_ms = 0;
                    }

                    bool turn_stable_done =
                        turn_in_tolerance &&
                        nav_turn_stable_since_ms > 0 &&
                        (now - nav_turn_stable_since_ms) >= NAV_TURN_STABLE_MS;

                    if (turn_stable_done) {
                        flag_nav_turn = false;
                        stop_motors();

                        tx_estring_value.clear();
                        tx_estring_value.append("TURN_DONE,");
                        tx_estring_value.append(current_yaw_deg);
                        tx_estring_value.append(",");
                        tx_estring_value.append(yaw_error_deg);
                        tx_characteristic_string.writeValue(tx_estring_value.c_str());
                    } else {
                        int turn_pwm = clamp_pwm_turn((int)fabs(u_cmd));

                        if (yaw_error_deg > 0.0f) {
                            turn_right_pwm(turn_pwm);
                        } else {
                            turn_left_pwm(turn_pwm);
                        }
                    }

                    log_nav_sample(NAV_MODE_TURN,
                                   now - nav_start_ms,
                                   current_yaw_deg,
                                   nav_target_yaw_deg,
                                   yaw_error_deg,
                                   -1.0f,
                                   actual_left_pwm,
                                   actual_right_pwm);

                    prev_yaw_error_deg = yaw_error_deg;
                }
            }

            if (flag_nav_drive) {
                unsigned long now = millis();
                unsigned long elapsed_ms = now - nav_start_ms;
                bool got_new_yaw = update_yaw_from_dmp();

                float drive_yaw_err = 0.0f;
                if (got_new_yaw && yaw_valid) {
                    drive_yaw_err = wrap_angle_deg(nav_drive_heading_deg - current_yaw_deg);
                }

                if (front.checkForDataReady()) {
                    nav_last_tof_mm = front.getDistance();
                    front.clearInterrupt();
                }

                if (nav_front_stop_mm > 0 &&
                    nav_last_tof_mm > 0.0f &&
                    nav_last_tof_mm <= nav_front_stop_mm) {
                    flag_nav_drive = false;
                    stop_motors();

                    tx_estring_value.clear();
                    tx_estring_value.append("DRIVE_STOPPED_TOF,");
                    tx_estring_value.append(nav_last_tof_mm);
                    tx_estring_value.append(",");
                    tx_estring_value.append((int)elapsed_ms);
                    tx_characteristic_string.writeValue(tx_estring_value.c_str());
                } else if (elapsed_ms >= (unsigned long)nav_drive_duration_ms) {
                    flag_nav_drive = false;
                    stop_motors();

                    tx_estring_value.clear();
                    tx_estring_value.append("DRIVE_DONE,");
                    tx_estring_value.append((int)elapsed_ms);
                    tx_estring_value.append(",");
                    tx_estring_value.append(current_yaw_deg);
                    tx_estring_value.append(",");
                    tx_estring_value.append(drive_yaw_err);
                    tx_characteristic_string.writeValue(tx_estring_value.c_str());
                } else {
                    int correction_pwm = (int)(nav_heading_kp * drive_yaw_err);
                    int left_base = nav_drive_base_pwm + correction_pwm;
                    int right_base = nav_drive_base_pwm - correction_pwm;

                    if (nav_drive_direction >= 0) {
                        drive_forward_split_pwm(left_base, right_base);
                    } else {
                        drive_backward_split_pwm(left_base, right_base);
                    }
                }

                log_nav_sample(NAV_MODE_DRIVE,
                               elapsed_ms,
                               current_yaw_deg,
                               nav_drive_heading_deg,
                               drive_yaw_err,
                               nav_last_tof_mm,
                               actual_left_pwm,
                               actual_right_pwm);
            }

            // Lab 6 orientation skeleton (DMP + PID 下一步再接)
            if (flag_orientation_pid) {
                unsigned long now = millis();
            
                bool got_new_yaw = update_yaw_from_dmp();
            
                if (got_new_yaw && yaw_valid) {
                    unsigned long dt_ms = now - last_orientation_ms;
                    last_orientation_ms = now;
            
                    float dt = dt_ms / 1000.0f;
                    if (dt <= 0.0f) dt = 0.001f;
            
                    yaw_error_deg = wrap_angle_deg(setpoint_yaw_deg - current_yaw_deg);

                    // P
                    p_term_ang = kp_ang * yaw_error_deg;
                    
                    // I
                    if (ki_ang == 0.0f) {
                        integral_yaw_error = 0.0f;
                    } else {
                        integral_yaw_error += yaw_error_deg * dt;
                    
                        if (integral_yaw_error > ANG_INTEGRAL_LIMIT) {
                            integral_yaw_error = ANG_INTEGRAL_LIMIT;
                        }
                        if (integral_yaw_error < -ANG_INTEGRAL_LIMIT) {
                            integral_yaw_error = -ANG_INTEGRAL_LIMIT;
                        }
                    }
                    i_term_ang = ki_ang * integral_yaw_error;
                    
                    // Use derivative on measurement to avoid setpoint-change kick.
                    float derivative_ang = 0.0f;
                    if (yaw_measurement_initialized) {
                        float delta_yaw_deg =
                            wrap_angle_deg(current_yaw_deg - prev_yaw_measurement_deg);
                        derivative_ang = -delta_yaw_deg / dt;
                    } else {
                        yaw_measurement_initialized = true;
                    }
                    prev_yaw_measurement_deg = current_yaw_deg;
                    
                    // 防止偶发尖峰
                    if (fabs(derivative_ang) > 1000.0f) {
                        derivative_ang = 0.0f;
                    }
                    
                    d_term_ang = kd_ang * derivative_ang;
                    
                    d_term_ang_filtered =
                        D_ANG_LPF_ALPHA * d_term_ang +
                        (1.0f - D_ANG_LPF_ALPHA) * last_d_term_ang_filtered;
                    
                    last_d_term_ang_filtered = d_term_ang_filtered;
                    
                    // PID
                    u_cmd = p_term_ang + i_term_ang + d_term_ang_filtered;
            
                    int turn_pwm = clamp_pwm_turn((int)fabs(u_cmd));
            
                    if (fabs(yaw_error_deg) <= YAW_TOL_DEG) {
                        u_cmd = 0.0f;
                        stop_motors();
                    } else {
                        if (yaw_error_deg > 0) {
                            turn_right_pwm(turn_pwm);
                        } else {
                            turn_left_pwm(turn_pwm);
                        }
                    }
            
                    log_orientation_sample(
                        now - orientation_start_ms,
                        current_yaw_deg,
                        setpoint_yaw_deg,
                        yaw_error_deg,
                        u_cmd,
                        i_term_ang,
                        d_term_ang_filtered,   // 这里开始记录 D
                        actual_left_pwm,
                        actual_right_pwm
                    );
            
                    prev_yaw_error_deg = yaw_error_deg;
                }
            
                if ((now - orientation_start_ms) >= orientation_test_duration_ms) {
                    flag_orientation_pid = false;
                    flag_orientation_log = false;
                    stop_motors();
            
                    tx_estring_value.clear();
                    tx_estring_value.append("ORIENTATION_PID_AUTO_STOPPED");
                    tx_characteristic_string.writeValue(tx_estring_value.c_str());
            
                    Serial.println("Orientation PID auto-stopped");
                }
            }

            if (flag_step_test) {
                unsigned long now = millis();

                // 保持恒定前进输入
                drive_forward_pwm(step_base_pwm_cmd);

                if (front.checkForDataReady()) {
                    float d_mm = front.getDistance();
                    front.clearInterrupt();

                    log_step_sample(now - step_start_ms,
                                    d_mm,
                                    step_base_pwm_cmd,
                                    actual_left_pwm,
                                    actual_right_pwm);

                    // 到安全距离或者数组满了就停
                    if (d_mm <= step_stop_dist_mm || step_idx >= MAX_STEP_LOG) {
                        flag_step_test = false;
                        stop_motors();

                        tx_estring_value.clear();
                        tx_estring_value.append("STEP_AUTO_STOPPED,");
                        tx_estring_value.append(d_mm);
                        tx_estring_value.append(",");
                        tx_estring_value.append(step_idx);
                        tx_characteristic_string.writeValue(tx_estring_value.c_str());

                        Serial.print("Step response auto-stopped. d_mm=");
                        Serial.print(d_mm);
                        Serial.print(", N=");
                        Serial.println(step_idx);
                    }
                }
            }

            if (flag_kf_pid) {
                unsigned long now = millis();
                bool got_new_tof = false;
                float new_raw = current_dist_mm;   

                if (front.checkForDataReady()) {
                    new_raw = front.getDistance();
                    front.clearInterrupt();
            
                    current_dist_mm = new_raw;
                    got_new_tof = true;
                }

                if (!kf_initialized && got_new_tof) {
                    reset_kf_state(new_raw, now);
                    last_tof_ms = now;
                    last_tof_dist_mm = new_raw;
                    tof_valid = true;
            
                    control_dist_mm = kf_est_dist_mm;

                    log_kf_sample(now - pid_start_ms,
                                  new_raw,
                                  kf_est_dist_mm,
                                  kf_est_vel_mm_s,
                                  0.0f);
                }

                if (kf_initialized) {
                    if (got_new_tof) {
                        run_kf_step(kf_last_u_scaled, new_raw, true);
            
                        last_tof_ms = now;
                        last_tof_dist_mm = new_raw;
                        tof_valid = true;
                    } else {
                        run_kf_step(kf_last_u_scaled, current_dist_mm, false);
                    }
            
                    // KF replaces linear extrapolation
                    control_dist_mm = kf_est_dist_mm;
            
                    unsigned long dt_ms = now - last_pid_ms;
                    last_pid_ms = now;
            
                    float dt = dt_ms / 1000.0f;
                    if (dt <= 0.0f) dt = 0.001f;
            
                    error_mm = control_dist_mm - target_mm;
            
                    // P
                    p_term = kp * error_mm;
            
                    // I
                    integral_error += error_mm * dt;
                    if (integral_error > INTEGRAL_LIMIT) integral_error = INTEGRAL_LIMIT;
                    if (integral_error < -INTEGRAL_LIMIT) integral_error = -INTEGRAL_LIMIT;
                    i_term = ki * integral_error;
            
                    // D
                    float derivative = (error_mm - prev_error_mm) / dt;
                    if (fabs(derivative) > 2000) derivative = 0.0f;
            
                    d_term = kd * derivative;
                    d_term_filtered = D_LPF_ALPHA * d_term +
                                      (1.0f - D_LPF_ALPHA) * last_d_term_filtered;
                    last_d_term_filtered = d_term_filtered;
            
                    // PID output
                    u_cmd = p_term + i_term + d_term_filtered;
            
                    if (error_mm > POS_TOL_MM) {
                        drive_forward_pwm((int)u_cmd);
                    }
                    else if (error_mm < -POS_TOL_MM) {
                        drive_backward_pwm((int)(-u_cmd));
                    }
                    else {
                        u_cmd = 0.0f;
                        stop_motors();
                    }
            
                    prev_error_mm = error_mm;
            
                    // Save the control actually being applied for NEXT KF prediction
                    kf_last_u_scaled = (float)clamp_pwm_base((int)fabs(u_cmd)) / 130.0f;
            
                    // Log raw + estimated
                    log_kf_sample(now - pid_start_ms,
                                  current_dist_mm,
                                  kf_est_dist_mm,
                                  kf_est_vel_mm_s,
                                  kf_last_u_scaled);
                }
            
                // same auto-stop rule as Lab 5
                if ((now - pid_start_ms) >= test_duration_ms) {
                    flag_kf_pid = false;
                    stop_motors();
            
                    tx_estring_value.clear();
                    tx_estring_value.append("KF_PID_LOG_AUTO_STOPPED");
                    tx_characteristic_string.writeValue(tx_estring_value.c_str());
            
                    Serial.println("KF PID log auto-stopped");
                }
            }

            // Prelab 最小版：固定时长记录 ToF 数据，不做真正电机控制
            if (flag_pid_active) {
                unsigned long now = millis();
            
                // 1) 如果有新的 ToF 数据，就更新 raw 数据和最近两次读数
                if (front.checkForDataReady()) {
                    float new_raw = front.getDistance();
                    front.clearInterrupt();
            
                    current_dist_mm = new_raw;
            
                    if (!tof_valid) {
                        last_tof_ms = now;
                        last_tof_dist_mm = new_raw;
                        tof_valid = true;
                        tof_pair_valid = false;
            
                        est_dist_mm = new_raw;
                        control_dist_mm = new_raw;
                    } else {
                        prev_tof_ms = last_tof_ms;
                        prev_tof_dist_mm = last_tof_dist_mm;
            
                        last_tof_ms = now;
                        last_tof_dist_mm = new_raw;
            
                        if (last_tof_ms > prev_tof_ms) {
                            tof_slope_mm_per_ms =
                                (last_tof_dist_mm - prev_tof_dist_mm) /
                                float(last_tof_ms - prev_tof_ms);
                            tof_pair_valid = true;
                        }
                    }
                }
            
                // 2) 只要拿到过 ToF，就每圈都运行一次控制
                if (tof_valid) {
                    if (tof_pair_valid) {
                        est_dist_mm = last_tof_dist_mm +
                                      tof_slope_mm_per_ms * float(now - last_tof_ms);
                    } else {
                        est_dist_mm = last_tof_dist_mm;
                    }
            
                    control_dist_mm = est_dist_mm;
            
                    unsigned long dt_ms = now - last_pid_ms;
                    last_pid_ms = now;
            
                    float dt = dt_ms / 1000.0f;
                    if (dt <= 0.0f) dt = 0.001f;
            
                    error_mm = control_dist_mm - target_mm;
            
                    p_term = kp * error_mm;

                    // I
                    integral_error += error_mm * dt;
                    if (integral_error > INTEGRAL_LIMIT) integral_error = INTEGRAL_LIMIT;
                    if (integral_error < -INTEGRAL_LIMIT) integral_error = -INTEGRAL_LIMIT;
                    
                    i_term = ki * integral_error;
                    
                    // D
                    float derivative = (error_mm - prev_error_mm) / dt;

                    if (fabs(derivative) > 2000) derivative = 0;
                    
                    d_term = kd * derivative;
                    
                    // LPF filter
                    d_term_filtered = D_LPF_ALPHA * d_term +
                                      (1.0f - D_LPF_ALPHA) * last_d_term_filtered;
                    
                    last_d_term_filtered = d_term_filtered;
                    
                    // PID output
                    u_cmd = p_term + i_term + d_term_filtered;
            
                    if (error_mm > POS_TOL_MM) {
                        drive_forward_pwm((int)u_cmd);
                    }
                    else if (error_mm < -POS_TOL_MM) {
                        drive_backward_pwm((int)(-u_cmd));
                    }
                    else {
                        u_cmd = 0.0f;
                        stop_motors();
                    }
            
                    prev_error_mm = error_mm;
            
                    log_pid_sample(now - pid_start_ms,
                                   current_dist_mm,
                                   est_dist_mm,
                                   error_mm,
                                   u_cmd,
                                   actual_left_pwm,
                                   actual_right_pwm);
                }
            
                if ((now - pid_start_ms) >= test_duration_ms) {
                    flag_pid_active = false;
                    flag_log_active = false;
                    stop_motors();
            
                    tx_estring_value.clear();
                    tx_estring_value.append("PID_LOG_AUTO_STOPPED");
                    tx_characteristic_string.writeValue(tx_estring_value.c_str());
            
                    Serial.println("PID log auto-stopped");
                }
            }

            if (flag_pid_rate_test) {
                unsigned long now = millis();
            
                // 保证至少拿到过一次 ToF
                if (front.checkForDataReady()) {
                    float new_raw = front.getDistance();
                    front.clearInterrupt();
            
                    current_dist_mm = new_raw;
            
                    if (!tof_valid) {
                        last_tof_ms = now;
                        last_tof_dist_mm = new_raw;
                        tof_valid = true;
                        tof_pair_valid = false;
                    } else {
                        prev_tof_ms = last_tof_ms;
                        prev_tof_dist_mm = last_tof_dist_mm;
            
                        last_tof_ms = now;
                        last_tof_dist_mm = new_raw;
            
                        if (last_tof_ms > prev_tof_ms) {
                            tof_slope_mm_per_ms =
                                (last_tof_dist_mm - prev_tof_dist_mm) /
                                float(last_tof_ms - prev_tof_ms);
                            tof_pair_valid = true;
                        }
                    }
                }
            
                if (tof_valid) {
                    if (pid_rate_idx < MAX_PID_RATE) {
                        pid_rate_t[pid_rate_idx] = micros();
                        pid_rate_idx++;
                    }
                }
            
                // 测 3 秒就够了，差不多得了
                if ((now - pid_start_ms) >= 3000) {
                    flag_pid_rate_test = false;
            
                    tx_estring_value.clear();
                    tx_estring_value.append("PID_RATE_AUTO_STOPPED");
                    tx_characteristic_string.writeValue(tx_estring_value.c_str());
            
                    Serial.println("PID rate test auto-stopped");
                }
            }

            BLE.poll();
        }

        // BLE 断开后的 hard stop
        flag_pid_active = false;
        flag_log_active = false;
        flag_orientation_pid = false;
        flag_orientation_log = false;
        flag_step_test = false;
        flag_kf_pid = false;
        flag_drift = false;
        flag_map_scan = false;
        flag_nav_turn = false;
        flag_nav_drive = false;
        drift_state = DRIFT_IDLE;
        map_scan_state = MAP_SCAN_IDLE;
        stop_motors();

        Serial.println("Disconnected");
    }
}
