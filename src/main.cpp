/* INCLUDES */
#include "main.hpp"

/* VARIABLES */

// Input file with IMU-related data
std::ofstream infile;

// Output file with Roll-Pitch-Yaw comparison
std::ofstream outfile;

// Gyroscope - angular velocity [rad/s]
float gx = 0.0;
float gy = 0.0;
float gz = 0.0;

// Accelerometer - angular acceleration [rad/s^2]
float ax = 0.0;
float ay = 0.0;
float az = 0.0;

// Gyroscope - angular velocity for Fusion Algorithm [deg/s]
float fusion_gx = 0.0;
float fusion_gy = 0.0;
float fusion_gz = 0.0;

// Accelerometer - angular acceleration for Fusion Algorithm [g]
float fusion_ax = 0.0;
float fusion_ay = 0.0;
float fusion_az = 0.0;

// Magnetometer - not used in this project
float mx = 0.0;
float my = 0.0;
float mz = 0.0;

// Roll-Pitch-Yaw calculated by IMU
float roll_imu  = 0.0;
float pitch_imu = 0.0;
float yaw_imu   = 0.0;

// Common
float precision = 6.0;

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

/* AHRS Madgwick-related variables */
float roll_mad_deg  = 0.0;
float pitch_mad_deg = 0.0;
float yaw_mad_deg   = 0.0;

float roll_mad_rad  = 0.0;
float pitch_mad_rad = 0.0;
float yaw_mad_rad   = 0.0;

/* AHRS Mahony-related variables */
float roll_mah_deg  = 0.0;
float pitch_mah_deg = 0.0;
float yaw_mah_deg   = 0.0;

float roll_mah_rad  = 0.0;
float pitch_mah_rad = 0.0;
float yaw_mah_rad   = 0.0;

/* FUNCTIONS */

/* Get data from the input file */
/* gx, gy, gz - gyroscope's raw data; ax, ay, az - accelerometer's raw data; *_imu - Euler angles calculated by the IMU */
void getDataFromInputFile(void) {
    infile >> gx >> gy >> gz >> ax >> ay >> az >> roll_imu >> pitch_imu >> yaw_imu;
}

/* Save headers in the output file */
void saveHeadersInOutputFile(void) {
    if (outfile.is_open()) {
        outfile << "Index,";
        outfile << "Angular velocity X [m/s], Angular velocity Y [m/s], Angular velocity Z [m/s],";                 // Gyroscope's raw data
        outfile << "Angular velocity X [rad/s], Angular velocity Y [rad/s], Angular velocity Z [rad/s],";
        outfile << "Angular acceleration X [rad/s^2],Angular acceleration Y [rad/s^2],Angular acceleration Z [rad/s^2],";
        outfile << "Angular acceleration X [m/s^2],Angular acceleration Y [m/s^2],Angular acceleration Z [m/s^2],"; // Accelerometer's raw data
        outfile << "IMU Roll [degrees], IMU Pitch [degrees],IMU Yaw [degrees],";
        outfile << "IMU Roll [rad], IMU Pitch [rad],IMU Yaw [rad],";
        outfile << "Fusion Roll [degrees], Fusion Pitch [degrees],Fusion Yaw [degrees],";
        outfile << "Fusion Roll [rad], Fusion Pitch [rad],Fusion Yaw [rad],";
        outfile << "Madgwick Roll [degrees], Madgwick Pitch [degrees],Madgwick Yaw [degrees],";
        outfile << "Madgwick Roll [rad], Madgwick Pitch [rad],Madgwick Yaw [rad],";
        outfile << "Mahony Roll [degrees], Mahony Pitch [degrees],Mahony Yaw [degrees]";
        outfile << "Mahony Roll [rad], Mahony Pitch [rad],Mahony Yaw [rad]" << std::endl;
    }
}

/* Save sample index, gyroscope's raw measurements and converted to rad/s in the output file */
void saveGyroInOutputFile(int sample_idx, float gx, float gy, float gz, float gx_rad, float gy_rad, float gz_rad) {
    if (outfile.is_open()) {
        outfile << sample_idx << ",";
        outfile << std::setprecision(precision) << std::fixed << gz     << "," << gy     << "," << gz     << ",";
        outfile << std::setprecision(precision) << std::fixed << gx_rad << "," << gy_rad << "," << gz_rad << ",";
    }
}

/* Save acceleromenter's raw measurements and converted to rad/s^2 in the output file */
void saveAccInOutputFile(float ax, float ay, float az, float ax_rad, float ay_rad, float az_rad) {
    if (outfile.is_open()) {
        outfile << std::setprecision(precision) << std::fixed << ax     << "," << ay     << "," << az     << ",";
        outfile << std::setprecision(precision) << std::fixed << ax_rad << "," << ay_rad << "," << az_rad << ",";
    }
}

/* Save IMU's raw Roll-Pitch-Yaw in the output file */
void saveIMUInOutputFile(float roll_imu, float pitch_imu, float yaw_imu) {
    if (outfile.is_open()) {
        outfile << std::setprecision(precision) << std::fixed << roll_imu << "," << pitch_imu << "," << yaw_imu << ",";
    }
}

/* Normalize Roll-Pitch-Yaw calculated by Fusion Algoritm */
void normalizeRollPitchYawFusion(float roll_rad, float pitch_rad, float yaw_rad) {
    roll_fus_rad  = fmod(roll_rad, (2*M_PI));
    pitch_fus_rad = fmod(pitch_rad, (2*M_PI));
    yaw_fus_rad   = fmod(yaw_rad, (2*M_PI));
}

/* Save Roll-Pitch-Yaw calculated by all Fusion Algorithm in the output file */
void saveRollPitchYawFusionInOutputFile (float roll_fus_deg, float pitch_fus_deg, float yaw_fus_deg, float roll_fus_rad, float pitch_fus_rad, float yaw_fus_rad) {
	if (outfile.is_open()) {
        outfile << std::setprecision(precision) << std::fixed << roll_fus_deg << "," << pitch_fus_deg << "," << yaw_fus_deg << ",";
        outfile << std::setprecision(precision) << std::fixed << roll_fus_rad << "," << pitch_fus_rad << "," << yaw_fus_rad << ",";
    }
}

/* Save Roll-Pitch-Yaw calculated by all Madgwick Algorithm in the output file */
void saveRollPitchYawMadgwickInOutputFile (float roll_mad_deg, float pitch_mad_deg, float yaw_mad_deg, float roll_mad_rad, float pitch_mad_rad, float yaw_mad_rad) {
	if (outfile.is_open()) {
        outfile << std::setprecision(precision) << std::fixed << roll_mad_deg << "," << pitch_mad_deg << "," << yaw_mad_deg << ",";
        outfile << std::setprecision(precision) << std::fixed << roll_mad_rad << "," << pitch_mad_rad << "," << yaw_mad_rad << ",";
    }
}

/* Save Roll-Pitch-Yaw calculated by Mahony Algorithm in the output file */
void saveRollPitchYawMahonyInOutputFile (float roll_mah_deg, float pitch_mah_deg, float yaw_mah_deg, float roll_mah_rad, float pitch_mah_rad, float yaw_mah_rad) {
	if (outfile.is_open()) {
        outfile << std::setprecision(precision) << std::fixed << roll_mah_deg << "," << pitch_mah_deg << "," << yaw_mah_deg << ",";
        outfile << std::setprecision(precision) << std::fixed << roll_mah_rad << "," << pitch_mah_rad << "," << yaw_mah_rad << std::endl;
    }
}

/* MAIN */
int main()
{
    /* Variables */

    // Initialize sample index
    int sample_idx = 1;
    std::string buffer = "2016-02-11-17-35-23";

    // Get input file
    std::fstream infile(buffer+"_gyro_acc_measurements.txt", std::ios_base::in);

    // Create output file with prefix of the input file
    outfile.open (buffer);

    getDataFromInputFile();
    saveHeadersInOutputFile();
    saveGyroInOutputFile(sample_idx, gx, gy, gz, gx_rad, gy_rad, gz_rad);
    saveAccInOutputFile(ax, ay, az, ax_rad, ay_rad, az_rad);
    saveIMUInOutputFile(roll_imu, pitch_imu, yaw_imu);

    /* AHRS FUSION-RELATED FUNCTIONS */

    // Initialise gyroscope bias correction algorithm
    FusionBiasInitialise(&fusionBias, 0.5f, sample_freq); // stationary threshold = 0.5 degrees per second

    // Initialise AHRS algorithm
    FusionAhrsInitialise(&fusionAhrs, 0.5f); // gain = 0.5

    /* MAIN LOOP */
    while (true)
    {
        /* AHRS FUSION-RELATED FUNCTIONS */
        fusion_gx  = FusionRadiansToDegrees(gx); // [deg/s]
        fusion_gy  = FusionRadiansToDegrees(gy); // [deg/s]
        fusion_gz  = FusionRadiansToDegrees(gz); // [deg/s]

	    // Calibrate gyroscope
        FusionVector3 uncalibratedGyroscope = {
            uncalibratedGyroscope.axis.x = fusion_gx,
            uncalibratedGyroscope.axis.y = fusion_gy,
            uncalibratedGyroscope.axis.z = fusion_gz,
        };
        FusionVector3 calibratedGyroscope = FusionCalibrationInertial(uncalibratedGyroscope, FUSION_ROTATION_MATRIX_IDENTITY, gyroscopeSensitivity, FUSION_VECTOR3_ZERO);

        // Accelometer - convert rad/s^2 to deg/s^2 and then to g
        convertAccForFusion(ax, ay, az); // [g]

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

        FusionEulerAngles eulerAngles = FusionQuaternionToEulerAngles(FusionAhrsGetQuaternion(&fusionAhrs));

        roll_fus_rad  = eulerAngles.angle.roll;  // [rad]
        pitch_fus_rad = eulerAngles.angle.pitch; // [rad]
        yaw_fus_rad   = eulerAngles.angle.yaw;   // [rad]

        normalizeRollPitchYawFusion(roll_fus_rad, pitch_fus_rad, yaw_fus_rad);

        roll_fus_deg  = FusionRadiansToDegrees(roll_fus_rad);  // [degrees]
        pitch_fus_deg = FusionRadiansToDegrees(pitch_fus_rad); // [degrees]
        yaw_fus_deg   = FusionRadiansToDegrees(yaw_fus_rad);   // [degrees]

        saveRollPitchYawFusionInOutputFile (roll_fus_deg, pitch_fus_deg, yaw_fus_deg, roll_fus_rad, pitch_fus_rad,  yaw_fus_rad);

        /* AHRS MADGWICK-RELATED FUNCTIONS */
        MadgwickGyroscopeAccelerometer(gx, gy, gz, ax, ay, az);
        computeAngles();

        roll_mad_rad  = roll;  // [rad]
	    pitch_mad_rad = pitch; // [rad]
	    yaw_mad_rad   = yaw;   // [rad]

        roll_mad_deg  = FusionRadiansToDegrees(roll_mad_rad);  // [degrees]
        pitch_mad_deg = FusionRadiansToDegrees(pitch_mad_rad); // [degrees]
        yaw_mad_deg   = FusionRadiansToDegrees(yaw_mad_rad);   // [degrees]

        saveRollPitchYawMadgwickInOutputFile (roll_mad_deg, pitch_mad_deg, yaw_mad_deg, roll_mad_rad, pitch_mad_rad, yaw_mad_rad);

        /* AHRS MAHONY-RELATED FUNCTIONS */
        MahonyGyroscopeAccelerometer(gx, gy, gz, ax, ay, az);
        computeAngles();

        roll_mah_rad  = roll;  // [rad]
	    pitch_mah_rad = pitch; // [rad]
	    yaw_mah_rad   = yaw;   // [rad]

        roll_mah_deg  = FusionRadiansToDegrees(roll_mah_rad);  // [degrees]
        pitch_mah_deg = FusionRadiansToDegrees(pitch_mah_rad); // [degrees]
        yaw_mah_deg   = FusionRadiansToDegrees(yaw_mah_rad);   // [degrees]

        saveRollPitchYawMahonyInOutputFile(roll_mah_deg, pitch_mah_deg, yaw_mah_deg, roll_mah_rad, pitch_mah_rad, yaw_mah_rad);

        std::cout << "Saving " << sample_idx << "th sample in the " << buffer << " file \r";

        // Increase sample index
        sample_idx++;
    } // END OF: while (true)
    outfile.close();
    infile.close();
    return;
} // END OF: main()
