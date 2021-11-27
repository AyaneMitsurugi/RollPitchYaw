/* INCLUDES */
#include "main.hpp"

/* VARIABLES */

// Input file with IMU-related data
std::fstream infile;

// Output file with Roll-Pitch-Yaw comparison
std::ofstream out_gyro_acc;
std::ofstream out_roll_pitch_yaw;
std::ofstream out_quaternion;

// Gyroscope [rad/s]
float gx_rad = 0.0;
float gy_rad = 0.0;
float gz_rad = 0.0;

// Accelerometer [rad/s^2]
float ax_rad = 0.0;
float ay_rad = 0.0;
float az_rad = 0.0;

// Gyroscope [deg/s]
float gx_deg = 0.0;
float gy_deg = 0.0;
float gz_deg = 0.0;

// Accelerometer [deg/s^2]
float ax_deg = 0.0;
float ay_deg = 0.0;
float az_deg = 0.0;

// Accelerometer for Fusion Algorithm [g]
float fusion_ax = 0.0;
float fusion_ay = 0.0;
float fusion_az = 0.0;

// Magnetometer - not used in this project
float mx = 0.0;
float my = 0.0;
float mz = 0.0;

// Roll-Pitch-Yaw calculated by IMU
float roll_imu_deg  = 0.0;
float pitch_imu_deg = 0.0;
float yaw_imu_deg   = 0.0;

float roll_imu_rad  = 0.0;
float pitch_imu_rad = 0.0;
float yaw_imu_rad   = 0.0;

float imu_qx = 0.0;
float imu_qy = 0.0;
float imu_qz = 0.0;
float imu_qw = 0.0;

// Common
float debug_print_en = 1.0;
float precision      = 6.0;
float sample_time    = 0.0;

/* AHRS Fusion-related variables */
FusionBias fusionBias;
FusionAhrs fusionAhrs;

FusionVector3 gyroscopeSensitivity = {
    gyroscopeSensitivity.axis.x = 2000.0f,
    gyroscopeSensitivity.axis.y = 2000.0f,
    gyroscopeSensitivity.axis.z = 2000.0f,
}; // replace these values with actual sensitivity in degrees per second per lsb as specified in gyroscope datasheet

FusionVector3 accelerometerSensitivity = {
    accelerometerSensitivity.axis.x = 16.0f,
    accelerometerSensitivity.axis.y = 16.0f,
    accelerometerSensitivity.axis.z = 16.0f,
}; // replace these values with actual sensitivity in g per lsb as specified in accelerometer datasheet

float roll_fus_deg  = 0.0;
float pitch_fus_deg = 0.0;
float yaw_fus_deg   = 0.0;

float roll_fus_rad  = 0.0;
float pitch_fus_rad = 0.0;
float yaw_fus_rad   = 0.0;

float fus_qx = 0.0;
float fus_qy = 0.0;
float fus_qz = 0.0;
float fus_qw = 0.0;

/* AHRS Madgwick-related variables */
float roll_mad_deg  = 0.0;
float pitch_mad_deg = 0.0;
float yaw_mad_deg   = 0.0;

float roll_mad_rad  = 0.0;
float pitch_mad_rad = 0.0;
float yaw_mad_rad   = 0.0;

float mad_qx = 0.0;
float mad_qy = 0.0;
float mad_qz = 0.0;
float mad_qw = 0.0;

/* AHRS Mahony-related variables */
float roll_mah_deg  = 0.0;
float pitch_mah_deg = 0.0;
float yaw_mah_deg   = 0.0;

float roll_mah_rad  = 0.0;
float pitch_mah_rad = 0.0;
float yaw_mah_rad   = 0.0;

float mah_qx = 0.0;
float mah_qy = 0.0;
float mah_qz = 0.0;
float mah_qw = 0.0;

/* FUNCTIONS */

/* Save headers in the output file */
void saveHeadersInGyroAccFile(void) {
    if (out_gyro_acc.is_open()) {
        out_gyro_acc << "Time [s],";
        out_gyro_acc << "Angular velocity X [deg/s], Angular velocity Y [deg/s], Angular velocity Z [deg/s],";
        out_gyro_acc << "Angular velocity X [rad/s], Angular velocity Y [rad/s], Angular velocity Z [rad/s],";
        out_gyro_acc << "Angular acceleration X [deg/s^2],Angular acceleration Y [deg/s^2],Angular acceleration Z [deg/s^2],";
        out_gyro_acc << "Angular acceleration X [rad/s^2],Angular acceleration Y [rad/s^2],Angular acceleration Z [rad/s^2]" << std::endl;
    }
}

void saveHeadersInRollPitchYawFile(void) {
    if (out_roll_pitch_yaw.is_open()) {
        out_roll_pitch_yaw << "Time [s],";
        out_roll_pitch_yaw << "IMU Roll [degrees], IMU Pitch [degrees],IMU Yaw [degrees],";
        out_roll_pitch_yaw << "IMU Roll [rad], IMU Pitch [rad],IMU Yaw [rad],";
        out_roll_pitch_yaw << "Fusion Roll [degrees], Fusion Pitch [degrees],Fusion Yaw [degrees],";
        out_roll_pitch_yaw << "Fusion Roll [rad], Fusion Pitch [rad],Fusion Yaw [rad],";
        out_roll_pitch_yaw << "Madgwick Roll [degrees], Madgwick Pitch [degrees],Madgwick Yaw [degrees],";
        out_roll_pitch_yaw << "Madgwick Roll [rad], Madgwick Pitch [rad],Madgwick Yaw [rad],";
        out_roll_pitch_yaw << "Mahony Roll [degrees], Mahony Pitch [degrees],Mahony Yaw [degrees]";
        out_roll_pitch_yaw << "Mahony Roll [rad], Mahony Pitch [rad],Mahony Yaw [rad]" << std::endl;
    }
}

void saveHeadersInQuaternionFile(void) {
    if (out_quaternion.is_open()) {
        out_quaternion << "Time [s],";
        out_quaternion << "IMU q_x, IMU q_y, IMU q_z, IMU q_w";
        out_quaternion << "Fusion q_x, Fusion q_y, Fusion q_z, Fusion q_w";
        out_quaternion << "Madgwick q_x, Madgwick q_y, Madgwick q_z, Madgwick q_w";
        out_quaternion << "Mahony q_x, Mahony q_y, Mahony q_z, Mahony q_w" << std::endl;
    }
}

/* Save gyroscope's and accelerometer's data in the output file */
void saveGyroAccInOutputFile(int sample_idx, float gx_deg, float gy_deg, float gz_deg, float gx_rad, float gy_rad, float gz_rad, float ax_deg, float ay_deg, float az_deg, float ax_rad, float ay_rad, float az_rad) {
    if (out_gyro_acc.is_open()) {
        out_gyro_acc << sample_idx << ",";
	    out_gyro_acc << std::setprecision(precision) << std::fixed << gx_deg << "," << gy_deg << "," << gz_deg << ",";
        out_gyro_acc << std::setprecision(precision) << std::fixed << gx_rad << "," << gy_rad << "," << gz_rad << ",";
	    out_gyro_acc << std::setprecision(precision) << std::fixed << ax_deg << "," << ay_deg << "," << az_deg << ",";
        out_gyro_acc << std::setprecision(precision) << std::fixed << ax_rad << "," << ay_rad << "," << az_rad << std::endl;
    }
}

/* Save IMU's Roll-Pitch-Yaw in the output file */
void saveIMUInOutputFile(int sample_idx, float roll_imu_deg, float pitch_imu_deg, float yaw_imu_deg, float roll_imu_rad, float pitch_imu_rad, float yaw_imu_rad) {
    if (out_roll_pitch_yaw.is_open()) {
        out_roll_pitch_yaw << sample_idx << ",";
        out_roll_pitch_yaw << std::setprecision(precision) << std::fixed << roll_imu_deg << "," << pitch_imu_deg << "," << yaw_imu_deg << ",";
        out_roll_pitch_yaw << std::setprecision(precision) << std::fixed << roll_imu_rad << "," << pitch_imu_rad << "," << yaw_imu_rad << ",";
    }
}

/* Save Roll-Pitch-Yaw calculated by all Fusion Algorithm in the output file */
void saveRollPitchYawFusionInOutputFile (float roll_fus_deg, float pitch_fus_deg, float yaw_fus_deg, float roll_fus_rad, float pitch_fus_rad, float yaw_fus_rad) {
	if (out_roll_pitch_yaw.is_open()) {
        out_roll_pitch_yaw << std::setprecision(precision) << std::fixed << roll_fus_deg << "," << pitch_fus_deg << "," << yaw_fus_deg << ",";
        out_roll_pitch_yaw << std::setprecision(precision) << std::fixed << roll_fus_rad << "," << pitch_fus_rad << "," << yaw_fus_rad << ",";
    }
}

/* Save Roll-Pitch-Yaw calculated by all Madgwick Algorithm in the output file */
void saveRollPitchYawMadgwickInOutputFile (float roll_mad_deg, float pitch_mad_deg, float yaw_mad_deg, float roll_mad_rad, float pitch_mad_rad, float yaw_mad_rad) {
	if (out_roll_pitch_yaw.is_open()) {
        out_roll_pitch_yaw << std::setprecision(precision) << std::fixed << roll_mad_deg << "," << pitch_mad_deg << "," << yaw_mad_deg << ",";
        out_roll_pitch_yaw << std::setprecision(precision) << std::fixed << roll_mad_rad << "," << pitch_mad_rad << "," << yaw_mad_rad << ",";
    }
}

/* Save Roll-Pitch-Yaw calculated by Mahony Algorithm in the output file */
void saveRollPitchYawMahonyInOutputFile (float roll_mah_deg, float pitch_mah_deg, float yaw_mah_deg, float roll_mah_rad, float pitch_mah_rad, float yaw_mah_rad) {
	if (out_roll_pitch_yaw.is_open()) {
        out_roll_pitch_yaw << std::setprecision(precision) << std::fixed << roll_mah_deg << "," << pitch_mah_deg << "," << yaw_mah_deg << ",";
        out_roll_pitch_yaw << std::setprecision(precision) << std::fixed << roll_mah_rad << "," << pitch_mah_rad << "," << yaw_mah_rad << std::endl;
    }
}

/* Save quaternions for all Algorithms in the output file */
void saveQuaternionsInOutputFile(int sample_idx, float imu_qx, float imu_qy, float imu_qz, float imu_qw, float fus_qx, float fus_qy, float fus_qz, float fus_qw, float mad_qx, float mad_qy, float mad_qz, float mad_qw, float mah_qx, float mah_qy, float mah_qz, float mah_qw) {
	if (out_quaternion.is_open()) {
        out_quaternion << sample_idx << ",";
        out_quaternion << std::setprecision(precision) << std::fixed << imu_qx << "," << imu_qy << "," << imu_qz << "," << imu_qw << ",";
        out_quaternion << std::setprecision(precision) << std::fixed << fus_qx << "," << fus_qy << "," << fus_qz << "," << fus_qw << ",";
        out_quaternion << std::setprecision(precision) << std::fixed << mad_qx << "," << mad_qy << "," << mad_qz << "," << mad_qw << ",";
        out_quaternion << std::setprecision(precision) << std::fixed << mah_qx << "," << mah_qy << "," << mah_qz << "," << mah_qw << std::endl;
    }
}

/* Convert deg/s^2 to g */
/* https://stackoverflow.com/questions/6291931/how-to-calculate-g-force-using-x-y-z-values-from-the-accelerometer-in-android/44421684 */
void convertAccForFusion(float ax_deg, float ay_deg, float az_deg) {
    const float g = 9.81;

    // Divide accelerometer's measurement by g = 9.81 to convert m/s^2 to g
    fusion_ax = ax_deg / g;
    fusion_ay = ay_deg / g;
    fusion_az = az_deg / g;
}

/* MAIN */
int main()
{
    /* Variables */
    std::fstream infile("../input_data/gyro_acc_measurements.txt", std::ios_base::in);

    out_gyro_acc.open("../output_data/gyro_acc");
    out_roll_pitch_yaw.open("../output_data/roll_pitch_yaw");
    out_quaternion.open("../output_data/quaternions");

    saveHeadersInGyroAccFile();
    saveHeadersInRollPitchYawFile();
    saveHeadersInQuaternionFile();

    /* AHRS FUSION-RELATED FUNCTIONS */
    // Initialise gyroscope bias correction algorithm
    FusionBiasInitialise(&fusionBias, 0.5f, sample_freq); // stationary threshold = 0.5 degrees per second

    // Initialise AHRS algorithm
    FusionAhrsInitialise(&fusionAhrs, 0.5f); // gain = 0.5

    std::cout << "Saving output file takes some time. Please be patient..." << std::endl;

    /* MAIN LOOP */
    while (infile >> sample_time >> gx_rad >> gy_rad >> gz_rad >> ax_rad >> ay_rad >> az_rad >> roll_imu_deg >> pitch_imu_deg >> yaw_imu_deg >> imu_qx >> imu_qy >> imu_qz >> imu_qw) {
        if (debug_print_en == 1) {
            std::cout << "\tINPUT FILE:" << std::endl;
            std::cout << "sample time = " << sample_time << std::endl;
            std::cout << "gx_rad = " << gx_rad << " gy_rad = " << gy_rad << " gz_rad = " << gz_rad << std::endl;
            std::cout << "ax_rad = "<< ax_rad << " ay_rad = " << ay_rad << " az_rad = " << az_rad << std::endl;
            std::cout << "roll_imu_deg = " << roll_imu_deg << " pitch_imu_deg = " << pitch_imu_deg << " yaw_imu_deg = " << yaw_imu_deg << std::endl;
            std::cout << "imu_qx = "<< imu_qx << " imu_qy = " << imu_qy << " imu_qz = " << imu_qz << " imu_qw = " << imu_qw << std::endl;
        }

        /* GYROSCOP{E */
        // Convert [rad/s] -> [deg/s] 
        gx_deg = FusionRadiansToDegrees(gx_rad);
        gy_deg = FusionRadiansToDegrees(gy_rad);
        gz_deg = FusionRadiansToDegrees(gz_rad);

        /* ACCELEROMETER */
        // Convert [rad/s^s] -> [deg/s^s]
        ax_deg = FusionRadiansToDegrees(ax_rad);
        ay_deg = FusionRadiansToDegrees(ay_rad);
        az_deg = FusionRadiansToDegrees(az_rad);
    
        saveGyroAccInOutputFile(sample_time, gx_deg, gy_deg, gz_deg, gx_rad, gy_rad, gz_rad, ax_deg, ay_deg, az_deg, ax_rad, ay_rad, az_rad);

        /* ROLL-PITCH-YAW CALCULATED BY IMU */
        // Convert [degrees] -> [rad]
        roll_imu_rad  = FusionDegreesToRadians(roll_imu_deg);
        pitch_imu_rad = FusionDegreesToRadians(pitch_imu_deg);
        yaw_imu_rad   = FusionDegreesToRadians(yaw_imu_deg);

        saveIMUInOutputFile(sample_time, roll_imu_deg, pitch_imu_deg, yaw_imu_deg, roll_imu_rad, pitch_imu_rad, yaw_imu_rad);

        if (debug_print_en == 1) {
            std::cout << "\tINPUT FILE AFTER unit conversion (rad -> degrees (gyro + acc) and degrees -> rad (Euler)):" << std::endl;
            std::cout << "gx_deg = " << gx_deg << " gy_deg = " << gy_deg << " gz_deg = " << gz_deg << std::endl;
            std::cout << "ax_deg = "<< ax_deg << " ay_deg = " << ay_deg << " az_deg = " << az_deg << std::endl;
            std::cout << "roll_imu_rad = " << roll_imu_rad << " pitch_imu_rad = "<< pitch_imu_rad << " yaw_imu_rad = " << yaw_imu_rad << std::endl;
        }

        /* AHRS FUSION-RELATED FUNCTIONS */

	    // Calibrate gyroscope
        FusionVector3 uncalibratedGyroscope = {
            uncalibratedGyroscope.axis.x = gx_deg,
            uncalibratedGyroscope.axis.y = gy_deg,
            uncalibratedGyroscope.axis.z = gz_deg,
        };
        FusionVector3 calibratedGyroscope = FusionCalibrationInertial(uncalibratedGyroscope, FUSION_ROTATION_MATRIX_IDENTITY, gyroscopeSensitivity, FUSION_VECTOR3_ZERO);

        // Convert [degrees/s^2] to [g-force]
        convertAccForFusion(ax_deg, ay_deg, az_deg);

	    // Calibrate accelerometer
        FusionVector3 uncalibratedAccelerometer = {
            uncalibratedAccelerometer.axis.x = fusion_ax,
            uncalibratedAccelerometer.axis.y = fusion_ay,
            uncalibratedAccelerometer.axis.z = fusion_az,
        };
        FusionVector3 calibratedAccelerometer = FusionCalibrationInertial(uncalibratedAccelerometer, FUSION_ROTATION_MATRIX_IDENTITY, accelerometerSensitivity, FUSION_VECTOR3_ZERO);

	    // Update gyroscope bias correction algorithm
        calibratedGyroscope = FusionBiasUpdate(&fusionBias, calibratedGyroscope);

        // Update AHRS algorithm
        FusionAhrsUpdateWithoutMagnetometer(&fusionAhrs, calibratedGyroscope, calibratedAccelerometer, sample_freq);

        // Quaternions
        auto FusionQuaternion = FusionAhrsGetQuaternion(&fusionAhrs);
        fus_qx = fusionAhrs.quaternion.element.x;
        fus_qy = fusionAhrs.quaternion.element.y;
        fus_qz = fusionAhrs.quaternion.element.z;
        fus_qw = fusionAhrs.quaternion.element.w;

        // Roll-Pitch-Yaw calculated by Fusion Algorithm
        FusionEulerAngles eulerAngles = FusionQuaternionToEulerAngles(FusionQuaternion);

        // Roll-Pitch-Yaw calculated by Fusion Algorithm
        roll_fus_deg  = eulerAngles.angle.roll;
        pitch_fus_deg = eulerAngles.angle.pitch;
        yaw_fus_deg   = eulerAngles.angle.yaw;

        // Convert [degrees] -> [rad]
        roll_fus_rad  = FusionDegreesToRadians(roll_fus_deg);
        pitch_fus_rad = FusionDegreesToRadians(pitch_fus_deg);
        yaw_fus_rad   = FusionDegreesToRadians(yaw_fus_deg);

        saveRollPitchYawFusionInOutputFile (roll_fus_deg, pitch_fus_deg, yaw_fus_deg, roll_fus_rad, pitch_fus_rad,  yaw_fus_rad);

        if (debug_print_en == 1) {
            std::cout << "\tFusion Algorithm:" << std::endl;
            std::cout << "roll_fus_deg = " << roll_fus_deg << " pitch_fus_deg = " << pitch_fus_deg << " yaw_fus_deg = " << yaw_fus_deg << std::endl;
            std::cout << "roll_fus_rad = " << roll_fus_rad << " pitch_fus_rad = " << pitch_fus_rad << " yaw_fus_rad = " << yaw_fus_rad << std::endl;
        }

        /* AHRS MADGWICK-RELATED FUNCTIONS */
        MadgwickGyroscopeAccelerometer(gx_rad, gy_rad, gz_rad, ax_rad, ay_rad, az_rad);
        computeAngles();

        // Quaternions
        mad_qx = q1;
        mad_qy = q2;
        mad_qz = q3;
        mad_qw = q0;

        // Roll-Pitch-Yaw calculated by Madgwick Algorithm
        roll_mad_rad  = roll;
	    pitch_mad_rad = pitch;
	    yaw_mad_rad   = yaw;

        // Convert [rad] -> [degrees]
        roll_mad_deg  = FusionRadiansToDegrees(roll_mad_rad);
        pitch_mad_deg = FusionRadiansToDegrees(pitch_mad_rad);
        yaw_mad_deg   = FusionRadiansToDegrees(yaw_mad_rad);

        saveRollPitchYawMadgwickInOutputFile (roll_mad_deg, pitch_mad_deg, yaw_mad_deg, roll_mad_rad, pitch_mad_rad, yaw_mad_rad);

        if (debug_print_en == 1) {
            std::cout << "\tMadgwick Algorithm:" << std::endl;
            std::cout << "roll_mad_deg = " << roll_mad_deg << " pitch_mad_deg = " << pitch_mad_deg << " yaw_mad_deg = " << yaw_mad_deg << std::endl;
            std::cout << "roll_mad_rad = " << roll_mad_rad << " pitch_mad_rad = " << pitch_mad_rad << " yaw_mad_rad = " << yaw_mad_rad << std::endl;
        }

        /* AHRS MAHONY-RELATED FUNCTIONS */
        MahonyGyroscopeAccelerometer(gx_rad, gy_rad, gz_rad, ax_rad, ay_rad, az_rad);
        computeAngles();

        // Quaternions
        mah_qx = q1;
        mah_qy = q2;
        mah_qz = q3;
        mah_qw = q0;

        // Roll-Pitch-Yaw calculated by Mahony Algorithm
        roll_mah_rad  = roll;
	    pitch_mah_rad = pitch;
	    yaw_mah_rad   = yaw;

        // Convert [rad] -> [degrees]
        roll_mah_deg  = FusionRadiansToDegrees(roll_mah_rad);
        pitch_mah_deg = FusionRadiansToDegrees(pitch_mah_rad);
        yaw_mah_deg   = FusionRadiansToDegrees(yaw_mah_rad);

        saveRollPitchYawMahonyInOutputFile(roll_mah_deg, pitch_mah_deg, yaw_mah_deg, roll_mah_rad, pitch_mah_rad, yaw_mah_rad);

        saveQuaternionsInOutputFile(sample_time, imu_qx, imu_qy, imu_qz, imu_qw, fus_qx, fus_qy, fus_qz, fus_qw, mad_qx, mad_qy, mad_qz, mad_qw, mah_qx, mah_qy, mah_qz, mah_qw);

        if (debug_print_en == 1) {
            std::cout << "\tMahony Algorithm:" << std::endl;
            std::cout << "roll_mah_deg = " << roll_mah_deg << " pitch_mah_deg = " << pitch_mah_deg << " yaw_mah_deg = " << yaw_mah_deg << std::endl;
            std::cout << "roll_mah_rad = " << roll_mah_rad << " pitch_mah_rad = " << pitch_mah_rad << " yaw_mah_rad = " << yaw_mah_rad << std::endl;
            std::cout << "\tQuaternions:" << std::endl;
            std::cout << "imu_qx = " << imu_qx << " imu_qy = " << imu_qy << " imu_qz = " << imu_qz << " imu_qw = " << imu_qw << std::endl;
            std::cout << "fus_qx = " << fus_qx << " fus_qy = " << fus_qy << " fus_qz = " << fus_qz << " fus_qw = " << fus_qw << std::endl;
            std::cout << "mad_qx = " << mad_qx << " mad_qy = " << mad_qy << " mad_qz = " << mad_qz << " mad_qw = " << mad_qw << std::endl;
            std::cout << "mah_qx = " << mah_qx << " mah_qy = " << mah_qy << " mah_qz = " << mah_qz << " mah_qw = " << mah_qw << std::endl;
        }
    } // END OF: while (true)
    std::cout << "Saving output file is COMPLETE!" << std::endl;
    infile.close();
    out_gyro_acc.close();
    out_roll_pitch_yaw.close();
    out_quaternion.close();
    return 0;
} // END OF: main()
