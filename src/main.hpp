/* INCLUDES */
#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include "AHRS/Fusion/Fusion.h"
#include "AHRS/Fusion/FusionAhrs.h"
#include "AHRS/Fusion/FusionBias.h"
#include "AHRS/Fusion/FusionCalibration.h"
#include "AHRS/Fusion/FusionTypes.h"
#include "AHRS/Madgwick/MadgwickAHRS.hpp"
#include "AHRS/Mahony/MahonyAHRS.hpp"
#include "AHRS/Common/CommonAHRS.hpp"

/* EXTERNAL VARIABLES */
// Gyroscope [rad/s]
extern float gx_rad;
extern float gy_rad;
extern float gz_rad;

// Accelerometer [rad/s^2]
extern float ax_rad;
extern float ay_rad;
extern float az_rad;

// Gyroscope [deg/s]
extern float gx_deg;
extern float gy_deg;
extern float gz_deg;

// Accelerometer [deg/s^2]
extern float ax_deg;
extern float ay_deg;
extern float az_deg;

// Accelerometer for Fusion Algorithm [g]
extern float fusion_ax;
extern float fusion_ay;
extern float fusion_az;

// Magnetometer - not used in this project
extern float mx;
extern float my;
extern float mz;

// Roll-Pitch-Yaw calculated by IMU
extern float roll_imu_deg;
extern float pitch_imu_deg;
extern float yaw_imu_deg;

extern float roll_imu_rad;
extern float pitch_imu_rad;
extern float yaw_imu_rad;

extern float imu_qx;
extern float imu_qy;
extern float imu_qz;
extern float imu_qw;

// Common
extern float debug_print_en;
extern float precision;
extern float sample_time;

extern float roll;
extern float pitch;
extern float yaw;

/* AHRS Fusion-related parameters */
extern FusionBias fusionBias;
extern FusionAhrs fusionAhrs;

extern FusionVector3 gyroscopeSensitivity;
extern FusionVector3 accelerometerSensitivity;

/* AHRS Fusion-related parameters */
extern float roll_fus_deg;
extern float pitch_fus_deg;
extern float yaw_fus_deg;

extern float roll_fus_rad;
extern float pitch_fus_rad;
extern float yaw_fus_rad;

extern float fus_qx;
extern float fus_qy;
extern float fus_qz;
extern float fus_qw;

/* AHRS Madgwick-related parameters */
extern float roll_mad_deg;
extern float pitch_mad_deg;
extern float yaw_mad_deg;

extern float roll_mad_rad;
extern float pitch_mad_rad;
extern float yaw_mad_rad;

extern float mad_qx;
extern float mad_qy;
extern float mad_qz;
extern float mad_qw;

/* AHRS Mahony-related functions */
extern float roll_mah_deg;
extern float pitch_mah_deg;
extern float yaw_mah_deg;

extern float roll_mah_rad;
extern float pitch_mah_rad;
extern float yaw_mah_rad;

extern float mah_qx;
extern float mah_qy;
extern float mah_qz;
extern float mah_qw;

/* FUNCTIONS */

/* Save headers in the output file */
void saveHeadersInGyroAccFile(void);

void saveHeadersInRollPitchYawFile(void);

void saveHeadersInQuaternionFile(void);

/* Save gyroscope's and accelerometer's data in the output file */
void saveGyroAccInOutputFile(int sample_idx, float gx_deg, float gy_deg, float gz_deg, float gx_rad, float gy_rad, float gz_rad, float ax_deg, float ay_deg, float az_deg, float ax_rad, float ay_rad, float az_rad);

/* Save IMU's Roll-Pitch-Yaw in the output file */
void saveIMUInOutputFile(int sample_idx, float roll_imu_deg, float pitch_imu_deg, float yaw_imu_deg, float roll_imu_rad, float pitch_imu_rad, float yaw_imu_rad);

/* Save Roll-Pitch-Yaw calculated by all Fusion Algorithm in the output file */
void saveRollPitchYawFusionInOutputFile (float roll_fus_deg, float pitch_fus_deg, float yaw_fus_deg, float roll_fus_rad, float pitch_fus_rad, float yaw_fus_rad);

/* Save Roll-Pitch-Yaw calculated by all Madgwick Algorithm in the output file */
void saveRollPitchYawMadgwickInOutputFile (float roll_mad_deg, float pitch_mad_deg, float yaw_mad_deg, float roll_mad_rad, float pitch_mad_rad, float yaw_mad_rad);

/* Save Roll-Pitch-Yaw calculated by Mahony Algorithm in the output file */
void saveRollPitchYawMahonyInOutputFile (float roll_mah_deg, float pitch_mah_deg, float yaw_mah_deg, float roll_mah_rad, float pitch_mah_rad, float yaw_mah_rad);

/* Save quaternions for all Algorithms in the output file */
void saveQuaternionsInOutputFile(int sample_idx, float imu_qx, float imu_qy, float imu_qz, float imu_qw, float fus_qx, float fus_qy, float fus_qz, float fus_qw, float mad_qx, float mad_qy, float mad_qz, float mad_qw, float mah_qx, float mah_qy, float mah_qz, float mah_qw);

/* Convert deg/s^2 to g */
/* https://stackoverflow.com/questions/6291931/how-to-calculate-g-force-using-x-y-z-values-from-the-accelerometer-in-android/44421684 */
void convertAccForFusion(float ax_deg, float ay_deg, float az_deg);