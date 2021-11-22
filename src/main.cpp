/* INCLUDES */
#include "main.hpp"

/* VARIABLES */

// Input file with IMU-related data
std::fstream infile;

// Output file with Roll-Pitch-Yaw comparison
std::ofstream outfile;

// Gyroscope [rad/s]
float gx = 0.0;
float gy = 0.0;
float gz = 0.0;

// Accelerometer [rad/s^2]
float ax = 0.0;
float ay = 0.0;
float az = 0.0;

// Gyroscope [m/s]
float gx_m = 0.0;
float gy_m = 0.0;
float gz_m = 0.0;

// Accelerometer [m/s^2]
float ax_m = 0.0;
float ay_m = 0.0;
float az_m = 0.0;

// Gyroscope for Fusion Algorithm [deg/s]
float fusion_gx = 0.0;
float fusion_gy = 0.0;
float fusion_gz = 0.0;

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

// Common
float debug_print_en = 0.0;
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

/* Save headers in the output file */
void saveHeadersInOutputFile(void) {
    if (outfile.is_open()) {
        outfile << "Time [s],";
        outfile << "Angular velocity X [m/s], Angular velocity Y [m/s], Angular velocity Z [m/s],";                 // Gyroscope's raw data
        outfile << "Angular velocity X [rad/s], Angular velocity Y [rad/s], Angular velocity Z [rad/s],";
        outfile << "Angular acceleration X [m/s^2],Angular acceleration Y [m/s^2],Angular acceleration Z [m/s^2],"; // Accelerometer's raw data
        outfile << "Angular acceleration X [rad/s^2],Angular acceleration Y [rad/s^2],Angular acceleration Z [rad/s^2],";
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

/* Save sample_time, gyroscope's raw measurements and converted to rad/s in the output file */
void saveGyroInOutputFile(float sample_time, float gx_m, float gy_m, float gz_m, float gx, float gy, float gz) {
    if (outfile.is_open()) {
        outfile << std::setprecision(precision) << std::fixed << sample_time << ",";
        outfile << std::setprecision(precision) << std::fixed << gx_m << "," << gy_m << "," << gz_m << ",";
        outfile << std::setprecision(precision) << std::fixed << gx   << "," << gy   << "," << gz   << ",";
    }
}

/* Save acceleromenter's raw measurements and converted to rad/s^2 in the output file */
void saveAccInOutputFile(float ax_m, float ay_m, float az_m, float ax, float ay, float az) {
    if (outfile.is_open()) {
        outfile << std::setprecision(precision) << std::fixed << ax_m << "," << ay_m << "," << az_m << ",";
        outfile << std::setprecision(precision) << std::fixed << ax   << "," << ay   << "," << az   << ",";
    }
}

/* Save IMU's Roll-Pitch-Yaw in the output file */
void saveIMUInOutputFile(float roll_imu_deg, float pitch_imu_deg, float yaw_imu_deg, float roll_imu_rad, float pitch_imu_rad, float yaw_imu_rad) {
    if (outfile.is_open()) {
        outfile << std::setprecision(precision) << std::fixed << roll_imu_deg << "," << pitch_imu_deg << "," << yaw_imu_deg << ",";
        outfile << std::setprecision(precision) << std::fixed << roll_imu_rad << "," << pitch_imu_rad << "," << yaw_imu_rad << ",";
    }
}

/* Convert rad/s^2 to g */
/* https://stackoverflow.com/questions/6291931/how-to-calculate-g-force-using-x-y-z-values-from-the-accelerometer-in-android/44421684 */
void convertAccForFusion(float ax, float ay, float az) {
    const float g = 9.81;

    // Firstly, convert rad/s^2 to deg/s^2
    fusion_ax = FusionRadiansToDegrees(ax);
    fusion_ay = FusionRadiansToDegrees(ay);
    fusion_az = FusionRadiansToDegrees(az);

    // Secondly, divide by g = 9.81 to convert deg/s^2 to g
    fusion_ax /= g;
    fusion_ay /= g;
    fusion_az /= g;
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
    std::string  file_name = "2016-02-11-17-45-07";
    std::fstream infile("../input_data/"+file_name+"_gyro_acc_measurements.txt", std::ios_base::in);

    // Create output file with the same prefix as an input file
    outfile.open("../output_data/"+file_name);

    saveHeadersInOutputFile();

    /* AHRS FUSION-RELATED FUNCTIONS */
    // Initialise gyroscope bias correction algorithm
    FusionBiasInitialise(&fusionBias, 0.5f, sample_freq); // stationary threshold = 0.5 degrees per second

    // Initialise AHRS algorithm
    FusionAhrsInitialise(&fusionAhrs, 0.5f); // gain = 0.5

    std::cout << "Saving output file takes some time. Please be patient..." << std::endl;

    /* MAIN LOOP */
    while (infile >> sample_time >> gx_m >> gy_m >> gz_m >> ax_m >> ay_m >> az_m >> roll_imu_deg >> pitch_imu_deg >> yaw_imu_deg) {
        if (debug_print_en == 1) {
            std::cout << "\tINPUT FILE:" << std::endl;
            std::cout << "sample time = " << sample_time << " gx_m = " << gx_m << " gy_m = " << gy_m << " gz_m = " << gz_m << std::endl;
            std::cout << "ax_m = "<< ax_m << " ay_m = " << ay_m << " az_m = " << az_m << std::endl;
            std::cout << "roll_imu_deg = " << roll_imu_deg << " pitch_imu_deg = "<< pitch_imu_deg << " yaw_imu_deg = " << yaw_imu_deg << std::endl;
        }

        /* GYROSCOP{E */
        // Convert [m/s] -> [rad/s]
        gx = FusionDegreesToRadians(gx_m);
        gy = FusionDegreesToRadians(gy_m);
        gz = FusionDegreesToRadians(gz_m);

        saveGyroInOutputFile(sample_time, gx_m, gy_m, gz_m, gx, gy, gz);

        /* ACCELEROMETER */
        // Convert [m/s^s] -> [rad/s^s]
        ax = FusionDegreesToRadians(ax_m);
        ay = FusionDegreesToRadians(ay_m);
        az = FusionDegreesToRadians(az_m);
    
        saveAccInOutputFile(ax_m, ay_m, az_m, ax, ay, az);

        /* ROLL-PITCH-YAW CALCULATED BY IMU */
        // Convert [degrees] -> [rad]
        roll_imu_rad  = FusionDegreesToRadians(roll_imu_deg);
        pitch_imu_rad = FusionDegreesToRadians(pitch_imu_deg);
        yaw_imu_rad   = FusionDegreesToRadians(yaw_imu_deg);

        saveIMUInOutputFile(roll_imu_deg, pitch_imu_deg, yaw_imu_deg, roll_imu_rad, pitch_imu_rad, yaw_imu_rad);

        if (debug_print_en == 1) {
            std::cout << "\tINPUT FILE AFTER unit conversion (m -> rad and deg -> rad):" << std::endl;
            std::cout << " gx = " << gx << " gy = " << gy << " gz = " << gz << std::endl;
            std::cout << "ax = "<< ax << " ay = " << ay << " az = " << az << std::endl;
            std::cout << "roll_imu_rad = " << roll_imu_rad << " pitch_imu_rad = "<< pitch_imu_rad << " yaw_imu_rad = " << yaw_imu_rad << std::endl;
        }

        /* AHRS FUSION-RELATED FUNCTIONS */
        // Convert [rad/s] -> [deg/s]
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

        // Convert [rad/s^2] to [deg/s^2] and then to [g]
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

        // Roll-Pitch-Yaw calculated by Fusion Algorithm
        FusionEulerAngles eulerAngles = FusionQuaternionToEulerAngles(FusionAhrsGetQuaternion(&fusionAhrs));

        roll_fus_rad  = eulerAngles.angle.roll;  // [rad]
        pitch_fus_rad = eulerAngles.angle.pitch; // [rad]
        yaw_fus_rad   = eulerAngles.angle.yaw;   // [rad]

        normalizeRollPitchYawFusion(roll_fus_rad, pitch_fus_rad, yaw_fus_rad);

        // Convert [rad] -> [degrees]
        roll_fus_deg  = FusionRadiansToDegrees(roll_fus_rad);  // [degrees]
        pitch_fus_deg = FusionRadiansToDegrees(pitch_fus_rad); // [degrees]
        yaw_fus_deg   = FusionRadiansToDegrees(yaw_fus_rad);   // [degrees]

        saveRollPitchYawFusionInOutputFile (roll_fus_deg, pitch_fus_deg, yaw_fus_deg, roll_fus_rad, pitch_fus_rad,  yaw_fus_rad);

        if (debug_print_en == 1) {
            std::cout << "\tFusion Algorithm:" << std::endl;
            std::cout << "roll_fus_deg = " << roll_fus_deg << " pitch_fus_deg = " << pitch_fus_deg << " yaw_fus_deg = " << yaw_fus_deg << std::endl;
            std::cout << "roll_fus_rad = " << roll_fus_rad << " pitch_fus_rad = " << pitch_fus_rad << " yaw_fus_rad = " << yaw_fus_rad << std::endl;
        }

        /* AHRS MADGWICK-RELATED FUNCTIONS */
        MadgwickGyroscopeAccelerometer(gx, gy, gz, ax, ay, az);
        computeAngles();

        // Roll-Pitch-Yaw calculated by Madgwick Algorithm
        roll_mad_rad  = roll;  // [rad]
	    pitch_mad_rad = pitch; // [rad]
	    yaw_mad_rad   = yaw;   // [rad]

        // Convert [rad] -> [degrees]
        roll_mad_deg  = FusionRadiansToDegrees(roll_mad_rad);  // [degrees]
        pitch_mad_deg = FusionRadiansToDegrees(pitch_mad_rad); // [degrees]
        yaw_mad_deg   = FusionRadiansToDegrees(yaw_mad_rad);   // [degrees]

        saveRollPitchYawMadgwickInOutputFile (roll_mad_deg, pitch_mad_deg, yaw_mad_deg, roll_mad_rad, pitch_mad_rad, yaw_mad_rad);

        if (debug_print_en == 1) {
            std::cout << "\tMadgwick Algorithm:" << std::endl;
            std::cout << "roll_mad_deg = " << roll_mad_deg << " pitch_mad_deg = " << pitch_mad_deg << " yaw_mad_deg = " << yaw_mad_deg << std::endl;
            std::cout << "roll_mad_rad = " << roll_mad_rad << " pitch_mad_rad = " << pitch_mad_rad << " yaw_mad_rad = " << yaw_mad_rad << std::endl;
        }

        /* AHRS MAHONY-RELATED FUNCTIONS */
        MahonyGyroscopeAccelerometer(gx, gy, gz, ax, ay, az);
        computeAngles();

        // Roll-Pitch-Yaw calculated by Mahony Algorithm
        roll_mah_rad  = roll;  // [rad]
	    pitch_mah_rad = pitch; // [rad]
	    yaw_mah_rad   = yaw;   // [rad]

        // Convert [rad] -> [degrees]
        roll_mah_deg  = FusionRadiansToDegrees(roll_mah_rad);  // [degrees]
        pitch_mah_deg = FusionRadiansToDegrees(pitch_mah_rad); // [degrees]
        yaw_mah_deg   = FusionRadiansToDegrees(yaw_mah_rad);   // [degrees]

        saveRollPitchYawMahonyInOutputFile(roll_mah_deg, pitch_mah_deg, yaw_mah_deg, roll_mah_rad, pitch_mah_rad, yaw_mah_rad);

        if (debug_print_en == 1) {
            std::cout << "\tMahony Algorithm:" << std::endl;
            std::cout << "roll_mah_deg = " << roll_mah_deg << " pitch_mah_deg = " << pitch_mah_deg << " yaw_mah_deg = " << yaw_mah_deg << std::endl;
            std::cout << "roll_mah_rad = " << roll_mah_rad << " pitch_mah_rad = " << pitch_mah_rad << " yaw_mah_rad = " << yaw_mah_rad << std::endl;
        }
    } // END OF: while (true)
    std::cout << "Saving output file is COMPLETE!" << std::endl;
    outfile.close();
    infile.close();
    return 0;
} // END OF: main()
