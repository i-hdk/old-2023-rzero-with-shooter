package raidzero.robot;

import java.nio.file.Path;

import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Constants {
    /**
     * Swerve Constants
     */
    public static final class SwerveConstants {
        public static final double kOpenLoopRampRate = 0.25;
        public static final double kClosedLoopRampRate = 0.0;
        /** Device IDs */
        public static final int FRONT_LEFT_THROTTLE_ID = 1;
        public static final int FRONT_RIGHT_THROTTLE_ID = 7;
        public static final int REAR_LEFT_THROTTLE_ID = 3;
        public static final int REAR_RIGHT_THROTTLE_ID = 5;

        public static final int FRONT_LEFT_ROTOR_ID = 2;
        public static final int FRONT_RIGHT_ROTOR_ID = 8;
        public static final int REAR_LEFT_ROTOR_ID = 4;
        public static final int REAR_RIGHT_ROTOR_ID = 6;

        public static final int FRONT_LEFT_ENCODER_ID = 1;
        public static final int FRONT_RIGHT_ENCODER_ID = 4;
        public static final int REAR_LEFT_ENCODER_ID = 2;
        public static final int REAR_RIGHT_ENCODER_ID = 3;

        public static final int IMU_ID = 0;

        // public static final double FRONT_LEFT_ROTOR_OFFSET = 118.828;
        // public static final double FRONT_RIGHT_ROTOR_OFFSET = 134.473;
        // public static final double REAR_LEFT_ROTOR_OFFSET = 268.066;
        // public static final double REAR_RIGHT_ROTOR_OFFSET = 349.980;

        public static final double FRONT_LEFT_ROTOR_OFFSET = 166.553;
        public static final double FRONT_RIGHT_ROTOR_OFFSET = 24.082;
        public static final double REAR_LEFT_ROTOR_OFFSET = 31.641;
        public static final double REAR_RIGHT_ROTOR_OFFSET = 63.281;

        public static final double THROTTLE_REDUCTION = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0);
        public static final double ROTOR_REDUCTION = (14.0 / 50.0) * (10.0 / 60.0);
        public static final double WHEEL_DIAMETER_METERS = 0.1016;
        // public static final double WHEEL_DIAMETER_METERS = 0.095;
        public static final double MAX_VEL_MPS = 4.959668;

        public static final double AUTO_BEANS = 19.9;

        // 20.75 OR 22.75 inches
        public static final double TRACKWIDTH_METERS = Units.inchesToMeters(22.75);
        public static final double WHEELBASE_METERS = Units.inchesToMeters(22.75);
        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                // Front left
                new Translation2d(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
                // Front right
                new Translation2d(TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0),
                // Back left
                new Translation2d(-TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
                // Back right
                new Translation2d(-TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0));
        /** 254 Pathing Constants (smooth): */
        public static final double MAX_DRIVE_VEL_MPS = MAX_VEL_MPS * 0.6;
        public static final double MAX_DRIVE_ACCEL_MPSPS = MAX_DRIVE_VEL_MPS * 1.25;
        public static final double MAX_ANGULAR_VEL_RPS = 1.2 * Math.PI;
        public static final double MAX_ANGULAR_ACCEL_RPSPS = MAX_ANGULAR_VEL_RPS * 2;
        /** 254 Pathing Constants (fast): */
        // public static final double MAX_DRIVE_VEL = MAX_VEL_MPS;
        // public static final double MAX_DRIVE_ACCEL = MAX_DRIVE_VEL / 0.2;
        // public static final double MAX_STEERING_VEL = Units.degreesToRadians(1000);

        /** 254 Module Constants */
        public static final int ROTOR_POSITION_PID_SLOT = 0;
        public static final double ROTOR_KP = 1;// .75
        public static final double ROTOR_KI = 0;
        public static final double ROTOR_KD = 5;// 15
        public static final int THROTTLE_VELOCITY_PID_SLOT = 0;
        public static final double THROTTLE_KP = 0.1;
        public static final double THROTTLE_KI = 0.0;
        public static final double THROTTLE_KD = 0.01;
        public static final double THROTTLE_KF = 1023 /
                (MAX_VEL_MPS /
                        (Math.PI * WHEEL_DIAMETER_METERS * THROTTLE_REDUCTION / 2048.0 * 10));
        /** 1678 Pathing Constants */
        public static final double XCONTROLLER_KP = 1.5;
        public static final double YCONTROLLER_KP = 1.5;
        public static final double THETACONTROLLER_KP = 0.5;
        public static final double THETACONTROLLER_KD = 0;
        public static final double XCONTROLLER_TOLERANCE = 0.1;
        public static final double YCONTROLLER_TOLERANCE = 0.1;
        public static final double THETACONTROLLER_TOLERANCE = Math.toRadians(5);

        /** AutoAim Constants */
        public static final double AA_XCONTROLLER_KP = 1.6;
        public static final double AA_YCONTROLLER_KP = 1.6;
        public static final double AA_THETACONTROLLER_KP = 1.0;
        public static final double AA_THETACONTROLLER_KD = 0.1;
        public static final double AA_XCONTROLLER_TOLERANCE = 0.01;
        public static final double AA_YCONTROLLER_TOLERANCE = 0.01;
        public static final double AA_THETACONTROLLER_TOLERANCE = Math.toRadians(0.2);

        // Using SDS 6.75 ratio
        public static final double THROTTLE_TICKS_TO_METERS = Math.PI * WHEEL_DIAMETER_METERS
                / (2048 * (1 / THROTTLE_REDUCTION));
        public static final double CANCODER_TO_DEGREES = 360.0 / 4096.0;
        public static final boolean MOTOR_INVERSION = false;
        public static final boolean ROTOR_INVERSION = true;
        public static final boolean ROTOR_INVERT_SENSOR_PHASE = true;
        /** Current Limits */
        public static final SupplyCurrentLimitConfiguration ROTOR_CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(
                true,
                25,
                40,
                0.1);
        public static final SupplyCurrentLimitConfiguration THROTTLE_CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(
                true,
                35,
                60,
                0.1);
    }

    public static final class DriveConstants {
        public static final Rotation2d STARTING_ROTATION = new Rotation2d(0.0);
        public static final Pose2d STARTING_POSE = new Pose2d(
                0.5,
                3.0,
                STARTING_ROTATION);
        // private static final double MAX_ACCEL_DISTANCE = 6.0 * Math.pow(TIMEOUT_S, 2);
        private static final double MAX_ACCEL_DISTANCE = 0.01;
        private static final double GYRO_ERROR_DEGREES_TIMEOUT = (0.4 / SECONDS_IN_MINUTE) * TIMEOUT_S;
        public static final double CONFIDENCE_TO_ERROR = 1.0;
        public static final Matrix<N3, N1> STATE_STDEVS_MATRIX = new MatBuilder<N3, N1>(
                Nat.N3(),
                Nat.N1())
                .fill(MAX_ACCEL_DISTANCE, MAX_ACCEL_DISTANCE, GYRO_ERROR_DEGREES_TIMEOUT);
        public static final Matrix<N3, N1> VISION_STDEVS_MATRIX = new MatBuilder<N3, N1>(
                Nat.N3(),
                Nat.N1())
                .fill(1.0, 1.0, 1.0);
    }

    public class PathingConstants {
        public static final int BASE_TRAJ_PERIOD_MS = 0;
        public static final int MIN_POINTS_IN_TALON = 10;
        public static final int TRANSMIT_PERIOD_MS = 20;
    }

    public static final class VisionConstants {
        public static final String NAME = "SmartDashboard";
        public static final String APRILTAGFAMILY = "tag16h5";
        private static final String APRILTAGFILENAME = "AprilTagPoses.json";
        public static final Path APRILTAGPATH = Filesystem.getDeployDirectory().toPath().resolve(APRILTAGFILENAME);
        private static final double CAMERAXDISPLACEMENT = 0.0772;
        private static final double CAMERAYDISPLACEMENT = 0.3429;
        // private static final double CAMERAZDISPLACEMENT = 0.56198;
        public static final Rotation2d[] CAMERAANGLES = { new Rotation2d(0), new Rotation2d(Math.PI) };

        public static final Pose2d[] CAMERALOCATIONS = {
                new Pose2d(CAMERAXDISPLACEMENT, -CAMERAYDISPLACEMENT, CAMERAANGLES[0]),
                new Pose2d(-CAMERAXDISPLACEMENT, -CAMERAYDISPLACEMENT, CAMERAANGLES[1]) };
        public static final Transform2d[] CAMERATRANSFORMS = { new Transform2d(CAMERALOCATIONS[0], new Pose2d()),
                new Transform2d(CAMERALOCATIONS[1], new Pose2d()) };

        public static final double ANGLEHISTSECS = 1.0;
        public static final double DISTANCETOLERANCE = 3.0;
        public static final double DISTANCEERRORFACTOR = 0.01;
        public static final double ANGLEERRORFACTOR = 1;
        // public static final Pose2d[] APRILTAG_POSE2DS = {new Pose2d(1, 1, new
        // Rotation2d(.5))};
        // public final Pose2d[] APRILTAG_POSE2DS =
        // JSONTools.GenerateAprilTagPoses(APRILTAGPATH);
        Path trajectoryFilePath = Filesystem.getDeployDirectory().toPath().resolve("paths/");
        public static final int IMU_ID = 0;

        public static final double CONE_PIXELS_TO_METERS = 0.001;

        public static final double MID_FIELD_X_POS = 8.3;
        public static final double MID_FIELD_Y_POS = 4.2;
        
        public static final double ADD_VISION_TOLERANCE = 1.0;
        public static final double DISTANCE_RESET_TOLERANCE = 3.0;
        public static final double SPEED_RESET_TOLERANCE = 0.5;
        public static final double OMEGA_RESET_TOLERANCE = 0.2;
        
        public static final int NUM_THREADS = 10;

        /**
         * Auto Alignment Constants
         */
        // Blue Alliance
        // public static final Pose2d BLL = new Pose2d(1.66, 4.57, Rotation2d.fromDegrees(155));
        // public static final Pose2d BLM = new Pose2d(1.85, 4.66, Rotation2d.fromDegrees(180));
        // public static final Pose2d BLR = new Pose2d(1.85, 4.08, Rotation2d.fromDegrees(180));
        // public static final Pose2d BML = new Pose2d(1.85, 3.53, Rotation2d.fromDegrees(180));
        // public static final Pose2d BMM = new Pose2d(1.85, 2.94, Rotation2d.fromDegrees(180));
        // public static final Pose2d BMR = new Pose2d(1.85, 2.34, Rotation2d.fromDegrees(180));
        // public static final Pose2d BRL = new Pose2d(1.85, 1.90, Rotation2d.fromDegrees(180));
        // public static final Pose2d BRM = new Pose2d(1.85, 1.36, Rotation2d.fromDegrees(180));
        // public static final Pose2d BRR = new Pose2d(1.85, 0.52, Rotation2d.fromDegrees(180));
        // // Red Alliance
        // public static final Pose2d RLL = new Pose2d(14.79, 0.97, Rotation2d.fromDegrees(-22));
        // public static final Pose2d RLM = new Pose2d(14.65, 0.94, Rotation2d.fromDegrees(0));
        // public static final Pose2d RLR = new Pose2d(14.65, 1.53, Rotation2d.fromDegrees(0));
        // public static final Pose2d RML = new Pose2d(14.65, 2.12, Rotation2d.fromDegrees(0));
        // public static final Pose2d RMM = new Pose2d(14.65, 2.64, Rotation2d.fromDegrees(0));
        // public static final Pose2d RMR = new Pose2d(14.65, 3.25, Rotation2d.fromDegrees(0));
        // public static final Pose2d RRL = new Pose2d(14.65, 3.82, Rotation2d.fromDegrees(0));
        // public static final Pose2d RRM = new Pose2d(14.65, 4.26, Rotation2d.fromDegrees(0));
        // public static final Pose2d RRR = new Pose2d(14.65, 4.72, Rotation2d.fromDegrees(0));
        // // Human Pickup Station
        // public static final Pose2d BL_LOAD = new Pose2d(15.73, 7.67, Rotation2d.fromDegrees(0));
        // public static final Pose2d BR_LOAD = new Pose2d(15.73, 5.99, Rotation2d.fromDegrees(0));
        // public static final Pose2d RL_LOAD = new Pose2d(0.79, 5.99, Rotation2d.fromDegrees(180));
        // public static final Pose2d RR_LOAD = new Pose2d(0.79, 7.67, Rotation2d.fromDegrees(180));

        //Blue Alliance
        public static final double BLL = 5.20;
        public static final double BLM = 4.66;
        public static final double BLR = 4.08;
        public static final double BML = 3.53;
        public static final double BMM = 2.94;
        public static final double BMR = 2.34;
        public static final double BRL = 1.90;
        public static final double BRM = 1.36;
        public static final double BRR = 0.52;
        
        //Red Alliance
        public static final double RLL = 1.03;
        public static final double RLM = 0.99;
        public static final double RLR = 1.46;
        public static final double RML = 2.17;
        public static final double RMM = 2.64;
        public static final double RMR = 3.23;
        public static final double RRL = 3.79;
        public static final double RRM = 4.26;
        public static final double RRR = 4.77;
    }

    public static final class ArmConstants {
        /** Arm Kinematics Constants */
        public static final double LOWER_ARM_LENGTH = 1.0; // in meters
        public static final double UPPER_ARM_LENGTH = 0.85;

        public static final double BASE_PIVOT_COG = 0.0; // in meters
        public static final double JOINT_COM = 0.0;

        public static final double LOWER_ARM_WEIGHT = 0.0; // in pounds
        public static final double UPPER_ARM_WEIGHT = 0.0;

        public static final double LOWER_ARM_MOI = 0.0;
        public static final double UPPER_ARM_MOI = 0.0;

        public static final int LINKAGES = 2;

        /**
         * Constants for arm Distal endpoint locations for different
         * arm positions. Constants are measured in meters
         */
        // Safe Human Pickup
        public static final double[] INTER_HUMAN_PICKUP_STATION = { 0.1, 0.7, 90 };
        public static final double[] INTER2_HUMAN_PICKUP_STATION = { 0.01, 1.4, 90 };
        public static final double[] HUMAN_PICKUP_STATION = { 0.50, 0.97, 160 };

        public static final double[] DIRECT_HUMAN_PICKUP_STATION = { 0.73, 1.09, 133 };

        //public static final double[] INTER_EXT_HUMAN_PICKUP_STATION = { 0.52, 0.79, -60 };
        public static final double[] INTER_EXT_HUMAN_PICKUP_STATION = { 0.16, 1.02, -60 };
        public static final double[] EXT_HUMAN_PICKUP_STATION = { 0.64, 0.98, -106 };

        public static final double[] INTER_GRID_LOW = { 0.0, 0.0 };
        public static final double[] GRID_LOW = { 0.6, 0.0 };

        public static final double[] INTER_GRID_MEDIUM = { 0.50, 1.15, 70 };
        public static final double[] GRID_MEDIUM = { 0.93, 1.01, 155 };

        public static final double[] INTER_GRID_HIGH = { 0.50, 1.25, 70 };
        public static final double[] GRID_HIGH = { 1.25, 1.29, 155 };

        public static final double[] INTER_AUTON_EXTENDED_GRID_HIGH = { 0.52, 1.25, 70 };
        public static final double[] AUTON_EXTENDED_GRID_HIGH = { 1.25, 1.30, 155 };

        public static final double[] INTER_AUTON_GRID_HIGH = { 0.52, 1.25, 70 };
        public static final double[] AUTON_GRID_HIGH = { 1.15, 1.30, 155 };

        public static final double[] INTER_REV_GRID_HIGH = { 0.52, 1.25, -70 };
        public static final double[] REV_GRID_HIGH = { 1.36, 1.28, -155 };

        public static final double[] INTER_CUBE_GRID_HIGH = { 0.30, 1.11, 70 };
        public static final double[] CUBE_GRID_HIGH = { 1.28, 1.14, 97 };
        public static final double[] INTER_TELE_CUBE_GRID_HIGH = { 0.30, 1.11, 70 };
        public static final double[] TELE_CUBE_GRID_HIGH = { 1.06, 1.15, 97 };

        public static final double[] INTER_CUBE_GRID_MEDIUM = { 0.30, 1.05, 70 };
        public static final double[] CUBE_GRID_MEDIUM = { 0.93, 0.85, 97 };
        public static final double[] INTER_TELE_CUBE_GRID_MEDIUM = { 0.30, 1.05, 70 };
        public static final double[] TELE_CUBE_GRID_MEDIUM = { 0.93, 0.79, 97 };

        public static final double[] INTER_FLOOR_INTAKE = { 0.84, 0.39, 45 };
        public static final double[] INTER2_FLOOR_INTAKE = { 1.12, 0.27, 90 };
        public static final double[] FLOOR_INTAKE = { 0.57, 0.02, 165 };

        // public static final double[] REV_CONE_FLOOR_INTAKE = { 0.60, 0.19, -147 };
        public static final double[] REV_CONE_FLOOR_INTAKE = { 0.60, 0.22, -135 };

        public static final double[] INTER_REV_FLIPPED_CONE_FLOOR_INTAKE = { 0.65, 0.19, -147 };
        public static final double[] REV_FLIPPED_CONE_FLOOR_INTAKE = { 0.70, -0.05, -97 };

        public static final double[] INTER_REV_CUBE_FLOOR_INTAKE = { 0.65, 0.19, -7 };
        public static final double[] REV_CUBE_FLOOR_INTAKE = { 0.70, -0.03, -125 };

        public static final double[] INTER_CUBE_DUMP = { 0.65, 0.19, -147 };
        public static final double[] CUBE_DUMP = { 0.42, 0.27, 133 };

        /**
         * Constants for a DC brushed motor.
         * nominal_voltage -- voltage at which the motor constants were measured
         * stall_torque -- current draw when stalled in Newton-meters
         * stall_current -- current draw when stalled in Amps
         * free_current -- current draw under no load in Amps
         * free_speed -- angular velocity under no load in RPM
         **/
        public static final double STALL_TORQUE = 2.6;
        public static final double STALL_CURRENT = 105;
        public static final double FREE_SPEED = 5676;
        public static final double BASE_PIVOT_GEAR_RATIO = 150;
        public static final double JOINT_GEAR_RATIO = 150;
        public static final double BASE_PIVOT_MOTORS = 2;
        public static final double JOINT_MOTORS = 2;
        public static final double GRAVITY = 9.81;

        /** Regular Constants */
        public static final int LOWER_LEADER_ID = 11;
        public static final int LOWER_FOLLOWER_ID = 12;
        public static final int UPPER_LEADER_ID = 13;
        public static final int UPPER_FOLLOWER_ID = 14;

        public static final boolean LOWER_MOTOR_INVERSION = false;
        public static final boolean UPPER_MOTOR_INVERSION = false;

        public static final int LOWER_CURRENT_LIMIT = 65;
        public static final int LOWER_STALL_CURRENT_LIMIT = 1;
        public static final int LOWER_RPM_LIMIT = 600;

        public static final int UPPER_CURRENT_LIMIT = 65;
        public static final int UPPER_STALL_CURRENT_LIMIT = 1;
        public static final int UPPER_RPM_LIMIT = 600;

        public static final SparkMaxLimitSwitch.Type LOWER_FORWARD_LIMIT_TYPE = SparkMaxLimitSwitch.Type.kNormallyOpen;
        public static final SparkMaxLimitSwitch.Type LOWER_REVERSE_LIMIT_TYPE = SparkMaxLimitSwitch.Type.kNormallyOpen;

        public static final SparkMaxLimitSwitch.Type UPPER_FORWARD_LIMIT_TYPE = SparkMaxLimitSwitch.Type.kNormallyOpen;
        public static final SparkMaxLimitSwitch.Type UPPER_REVERSE_LIMIT_TYPE = SparkMaxLimitSwitch.Type.kNormallyOpen;

        public static final double LOWER_ZERO_OFFSET = 5.9092043;
        public static final double UPPER_ZERO_OFFSET = 0.0;

        public static final double LOWER_ABS_POSITION_CONVERSION_FACTOR = 6.283;
        public static final double UPPER_ABS_POSITION_CONVERSION_FACTOR = 0.0;

        public static final boolean LOWER_ENCODER_INVERSION = false;
        public static final boolean UPPER_ENCODER_INVERSION = false;
        public static final boolean LOWER_ABSOLUTE_ENCODER_INVERSION = true;
        public static final boolean UPPER_ABSOLUTE_ENCODER_INVERSION = false;

        public static final int LOWER_SMART_MOTION_SLOT = 0;
        public static final int UPPER_SMART_MOTION_SLOT = 0;

        public static final double TICKS_TO_DEGREES = 2.45;
        public static final double LOWER_MAX_ANGLE = 52.0;
        public static final double UPPER_FWD_SOFTLIMIT = 70;
        public static final double UPPER_REV_SOFTLIMIT = -70;
        public static final double X_EXTENSION_LIMIT = 1.4;
        public static final double Y_EXTENSION_LIMIT = 1.4;

        public static final double LOWER_KF = 0.000166;
        public static final double LOWER_KP = 0.0000727;
        public static final double LOWER_KI = 0.0;
        public static final double LOWER_KD = 0.00000;
        public static final double LOWER_MIN_VEL = 0.0;
        public static final double LOWER_MAX_VEL = 5020;
        public static final double LOWER_MAX_ACCEL = 3580;
        public static final double LOWER_MIN_ERROR = 0.0;

        public static final double UPPER_KF = 0.000166;
        public static final double UPPER_KP = 0.0000727;
        public static final double UPPER_KI = 0.0;
        public static final double UPPER_KD = 0.00000;
        public static final double UPPER_MIN_VEL = 0.0;
        public static final double UPPER_MAX_VEL = 7900;
        public static final double UPPER_MAX_ACCEL = 6100;
        public static final double UPPER_MIN_ERROR = 0.0;

        // Testing Speeds
        // public static final double LOWER_KF = 0.000166;
        // public static final double LOWER_KP = 0.000156;
        // public static final double LOWER_KI = 0.0;
        // public static final double LOWER_KD = 0.00000;
        // public static final double LOWER_MIN_VEL = 0.0;
        // public static final double LOWER_MAX_VEL = 700;
        // public static final double LOWER_MAX_ACCEL = 350;
        // public static final double LOWER_MIN_ERROR = 0.0;

        // public static final double UPPER_KF = 0.000166;
        // public static final double UPPER_KP = 0.000156;
        // public static final double UPPER_KI = 0.0;
        // public static final double UPPER_KD = 0.00000;
        // public static final double UPPER_MIN_VEL = 0.0;
        // public static final double UPPER_MAX_VEL = 1500;
        // public static final double UPPER_MAX_ACCEL = 700;
        // public static final double UPPER_MIN_ERROR = 0.0;

        public static final double TOTAL_MAX_ACCEL = LOWER_MAX_ACCEL + UPPER_MAX_ACCEL;
        public static final double TOTAL_MAX_VEL = LOWER_MAX_VEL + UPPER_MAX_VEL;

        public static final double PID_WRAPPING_MIN = 0.0;
        public static final double PID_WRAPPING_MAX = 360.0;
        public static final boolean UPPER_LIMIT_ENABLED = false;
    }

    public static final class WristConstants {
        public static final int ID = 15;

        public static final boolean INVERSION = true;

        public static final int CURRENT_LIMIT = 25;

        // public static final boolean ENCODER_INVERSION = false;
        // 1:75 ratio, in degrees
        public static final double GEAR_RATIO = 82.2;
        public static final double POSITION_CONVERSION_FACTOR = 1.0 / GEAR_RATIO * 360.0;
        public static final double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR / 60;

        public static final boolean ENABLEREVERSELIMIT = true;
        public static final boolean ENABLEFORWARDLIMIT = true;
        public static final float FORWARDLIMIT = 50;
        public static final float REVERSELIMIT = -50;

        public static final int SMART_MOTION_SLOT = 0;

        public static final double KF = 0.00009;
        public static final double KP = 0.000097;
        public static final double KI = 0.0;
        public static final double KD = 0.0;

        public static final double MIN_VEL = 0.0;
        public static final double MAX_VEL = 7270.0;
        public static final double MAX_ACCEL = 6900.0;
        public static final double MIN_ERROR = 0.0;

        public static final double MAXWINDS = 0.6;
        public static final double PID_WRAPPING_MIN = 0.0;
        public static final double PID_WRAPPING_MAX = 360.0;

        public static final SparkMaxLimitSwitch.Type LIMITSWITCHPOLARITY = SparkMaxLimitSwitch.Type.kNormallyOpen;
        public static final double LIMITSWITCHPOSITIONS[] = { 35.5, -22.5 };
        public static final double LIMITSWITCHDIFFERENCE = LIMITSWITCHPOSITIONS[1] - LIMITSWITCHPOSITIONS[0];

        public static final int LIMITSWITCHBUFFERSIZE = 100;

        public static final double ENCODER_NORMALIZATION = 100.0;

        public static final String NAME = null;
    }

    public static final class IntakeConstants {
        public static final int ID = 16;

        public static final boolean INVERSION = true;

        public static final int STALL_CURRENT_LIMIT = 15;
        public static final int FREE_CURRENT_LIMIT = 35;
        public static final int STALL_RPM = 100;
        public static final int PID_SLOT = 0;
        public static final int SMART_MOTION_SLOT = 0;
        public static final double KF = 0.00005;
        public static final double KP = 0.1; // 0.00005
        public static final double KI = 0.0;
        public static final double KD = 0.0;

        public static final double MIN_VEL = 0.0;
        public static final double MAX_VEL = 3000.0;
        public static final double MAX_ACCEL = 4500.0;
        public static final double MIN_ERROR = 3.0;

        public static final double AUTON_CONE_SCORE = -0.45;
        public static final double AUTON_CUBE_INTAKE = -0.7;
        public static final double AUTON_CUBE_SCORE = 0.35;

        public static final double MAXWINDS = 0.6;
    }

    public static final class TOFSensorConstants {
        public static final int SENSOR_ID = 0;
    }

    public static final class LightsConstants {
        public static final int CANDLE_ID = 0;
        public static final boolean LOS_BEHAVIOR = true;
        public static final LEDStripType LED_STRIP_TYPE = LEDStripType.GRB;
        public static final double BRIGHTNESS_SCALAR = 1.0;
        public static final boolean STATUS_LED_CONFIG = false;
        public static final VBatOutputMode V_BAT_OUTPUT_MODE = VBatOutputMode.Off;

        public static final int PRIMARY_ANIMATION_SLOT = 0;
    }

    public static final String NETWORKTABLESNAME = "SmartDashboard";

    public static final double JOYSTICK_DEADBAND = 0.02;
    public static final double XBOX_JOYSTICK_DEADBAND = 0.10;
    public static final double AIMING_JOYSTICK_DEADBAND = 0.02;
    public static final int TIMEOUT_MS = 20;
    public static final double TIMEOUT_S = TIMEOUT_MS / 1000.0f;
    public static final int SECONDS_IN_MINUTE = 60;
    public static final double SQRTTWO = Math.sqrt(2);
    public static final String CANBUS_STRING = "seCANdary";
    public static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE = PneumaticsModuleType.REVPH;
    public static final double VOLTAGE_COMP = 12.0;
}
