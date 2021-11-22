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
extern float gx;
extern float gy;
extern float gz;

// Accelerometer [rad/s^2]
extern float ax;
extern float ay;
extern float az;

// Gyroscope [m/s]
extern float gx_m[];
extern float gy_m[];
extern float gz_m[];

// Accelerometer [m/s^2]
extern float ax_m;
extern float ay_m;
extern float az_m;

// Gyroscope for Fusion Algorithm [deg/s]
extern float fusion_gx;
extern float fusion_gy;
extern float fusion_gz;

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

// Common
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

/* AHRS Madgwick-related parameters */
extern float roll_mad_deg;
extern float pitch_mad_deg;
extern float yaw_mad_deg;

extern float roll_mad_rad;
extern float pitch_mad_rad;
extern float yaw_mad_rad;

/* AHRS Mahony-related functions */
extern float roll_mah_deg;
extern float pitch_mah_deg;
extern float yaw_mah_deg;

extern float roll_mah_rad;
extern float pitch_mah_rad;
extern float yaw_mah_rad;

/* FUNCTIONS */

/* Get data from the input file */
/* gx, gy, gz - gyroscope's raw data; ax, ay, az - accelerometer's raw data; *_imu - Euler angles calculated by the IMU */
void getDataFromInputFile(void);

/* Save headers in the output file */
void saveHeadersInOutputFile(void);

/* Save sample_time, gyroscope's raw measurements and converted to rad/s in the output file */
void saveGyroInOutputFile(float sample_time, float gx_m, float gy_m, float gz_m, float gx, float gy, float gz);

/* Save acceleromenter's raw measurements and converted to rad/s^2 in the output file */
void saveAccInOutputFile(float ax_m, float ay_m, float az_m, float ax, float ay, float az);

/* Save IMU's Roll-Pitch-Yaw in the output file */
void saveIMUInOutputFile(float roll_imu_deg, float pitch_imu_deg, float yaw_imu_deg, float roll_imu_rad, float pitch_imu_rad, float yaw_imu_rad);

/* Convert rad/s^2 to g */
/* https://stackoverflow.com/questions/6291931/how-to-calculate-g-force-using-x-y-z-values-from-the-accelerometer-in-android/44421684 */
void convertAccForFusion(float ax, float ay, float az);

/* Normalize Roll-Pitch-Yaw calculated by Fusion Algoritm */
void normalizeRollPitchYawFusion(float roll_rad, float pitch_rad, float yaw_rad);

/* Save Roll-Pitch-Yaw calculated by all Fusion Algorithm in the output file */
void saveRollPitchYawFusionInOutputFile (float roll_fus_deg, float pitch_fus_deg, float yaw_fus_deg, float roll_fus_rad, float pitch_fus_rad, float yaw_fus_rad);

/* Save Roll-Pitch-Yaw calculated by all Madgwick Algorithm in the output file */
void saveRollPitchYawMadgwickInOutputFile (float roll_mad_deg, float pitch_mad_deg, float yaw_mad_deg, float roll_mad_rad, float pitch_mad_rad, float yaw_mad_rad);

/* Save Roll-Pitch-Yaw calculated by Mahony Algorithm in the output file */
void saveRollPitchYawMahonyInOutputFile (float roll_mah_deg, float pitch_mah_deg, float yaw_mah_deg, float roll_mah_rad, float pitch_mah_rad, float yaw_mah_rad);