package raidzero.robot.submodules;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
//import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
//import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import raidzero.robot.Constants;
import raidzero.robot.Constants.DriveConstants;
import raidzero.robot.Constants.SwerveConstants;
import raidzero.robot.dashboard.Tab;
import raidzero.robot.utils.AutoAimController;
import raidzero.robot.utils.AutoAimController.AutoAimLocation;

public class Swerve extends Submodule {
    private enum ControlState {
        OPEN_LOOP, PATHING, AUTO_AIM
    };

    private class WPI_Pigeon2_Helper extends WPI_Pigeon2 {
        public WPI_Pigeon2_Helper(int deviceNumber, String canbus) {
            super(deviceNumber, canbus);
        }

        public double getAngle() {
            return -super.getAngle();
        }
    }

    private static Swerve instance = null;

    public static Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }

    private Swerve() {
    }

    private static final Vision vision = Vision.getInstance();

    private SwerveModule topRightModule = new SwerveModule();
    private SwerveModule topLeftModule = new SwerveModule();
    private SwerveModule bottomLeftModule = new SwerveModule();
    private SwerveModule bottomRightModule = new SwerveModule();

    private WPI_Pigeon2_Helper pigeon;

    private SwerveDrivePoseEstimator odometry;

    private Pose2d currentPose;
    private Pose2d prevPose;
    private Field2d fieldPose = new Field2d();

    private PathPlannerTrajectory currentTrajectory;
    private boolean firstPath = true;
    private boolean overLimit = false;
    // private Rotation2d targetAngle;
    private PIDController xController, yController, thetaController;
    private Timer timer = new Timer();
    private Alliance alliance;
    private double beans;
    private double prevX;

    private Pose2d desiredAutoAimPose;
    private PIDController autoAimXController, autoAimYController;
    private ProfiledPIDController autoAimThetaController, snapController;
    private TrajectoryConfig autoAimTrajectoryConfig;
    private AutoAimController autoAimController;

    private ControlState controlState = ControlState.OPEN_LOOP;

    public void onStart(double timestamp) {
        controlState = ControlState.OPEN_LOOP;
        alliance = DriverStation.getAlliance();
        zero();
        firstPath = true;

        // TEMPORARY
        // setPose(new Pose2d(4,1.2,Rotation2d.fromDegrees(180)));
    }

    public void onInit() {
        pigeon = new WPI_Pigeon2_Helper(SwerveConstants.IMU_ID, Constants.CANBUS_STRING);
        Shuffleboard.getTab(Tab.MAIN).add("Pigey", pigeon).withSize(2, 2).withPosition(4, 4);

        topLeftModule.onInit(
                SwerveConstants.FRONT_LEFT_THROTTLE_ID,
                SwerveConstants.FRONT_LEFT_ROTOR_ID,
                SwerveConstants.FRONT_LEFT_ENCODER_ID,
                SwerveConstants.FRONT_LEFT_ROTOR_OFFSET);
        topRightModule.onInit(
                SwerveConstants.FRONT_RIGHT_THROTTLE_ID,
                SwerveConstants.FRONT_RIGHT_ROTOR_ID,
                SwerveConstants.FRONT_RIGHT_ENCODER_ID,
                SwerveConstants.FRONT_RIGHT_ROTOR_OFFSET);
        bottomLeftModule.onInit(
                SwerveConstants.REAR_LEFT_THROTTLE_ID,
                SwerveConstants.REAR_LEFT_ROTOR_ID,
                SwerveConstants.REAR_LEFT_ENCODER_ID,
                SwerveConstants.REAR_LEFT_ROTOR_OFFSET);
        bottomRightModule.onInit(
                SwerveConstants.REAR_RIGHT_THROTTLE_ID,
                SwerveConstants.REAR_RIGHT_ROTOR_ID,
                SwerveConstants.REAR_RIGHT_ENCODER_ID,
                SwerveConstants.REAR_RIGHT_ROTOR_OFFSET);

        odometry = new SwerveDrivePoseEstimator(
                SwerveConstants.KINEMATICS,
                Rotation2d.fromDegrees(pigeon.getAngle()),
                getModulePositions(),
                DriveConstants.STARTING_POSE,
                DriveConstants.STATE_STDEVS_MATRIX,
                DriveConstants.VISION_STDEVS_MATRIX);

        snapController = new ProfiledPIDController(1.25, 0, 0.15, new TrapezoidProfile.Constraints(
                SwerveConstants.MAX_ANGULAR_VEL_RPS, SwerveConstants.MAX_ANGULAR_ACCEL_RPSPS * 2));
        snapController.enableContinuousInput(-Math.PI, Math.PI);
        snapController.reset(getPose().getRotation().getRadians(), 0);

        xController = new PIDController(SwerveConstants.XCONTROLLER_KP, 0, 0);
        yController = new PIDController(SwerveConstants.YCONTROLLER_KP, 0, 0);
        thetaController = new PIDController(SwerveConstants.THETACONTROLLER_KP, 0, SwerveConstants.THETACONTROLLER_KD);
        xController.setTolerance(SwerveConstants.XCONTROLLER_TOLERANCE);
        yController.setTolerance(SwerveConstants.YCONTROLLER_TOLERANCE);
        thetaController.setTolerance(SwerveConstants.THETACONTROLLER_TOLERANCE);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        autoAimXController = new PIDController(SwerveConstants.AA_XCONTROLLER_KP, 0.0, 0.0);
        autoAimYController = new PIDController(SwerveConstants.AA_YCONTROLLER_KP, 0.0, 0.0);
        autoAimThetaController = new ProfiledPIDController(SwerveConstants.AA_THETACONTROLLER_KP, 0.0, 0.0,
                new TrapezoidProfile.Constraints(SwerveConstants.MAX_ANGULAR_VEL_RPS,
                        SwerveConstants.MAX_ANGULAR_ACCEL_RPSPS));
        autoAimTrajectoryConfig = new TrajectoryConfig(SwerveConstants.MAX_DRIVE_VEL_MPS,
                SwerveConstants.MAX_DRIVE_ACCEL_MPSPS);
        autoAimController = new AutoAimController(autoAimXController, autoAimYController, autoAimThetaController,
                autoAimTrajectoryConfig);
        autoAimController.setTolerance(new Pose2d(
                SwerveConstants.AA_XCONTROLLER_TOLERANCE,
                SwerveConstants.AA_YCONTROLLER_TOLERANCE,
                Rotation2d.fromRadians(SwerveConstants.AA_THETACONTROLLER_TOLERANCE)));

        zero();

        // Auto Balance Constants
        prevPose = new Pose2d();
        prevX = 0;

        PathPlannerServer.startServer(5811);
    }

    String control_state = "nada";

    @Override
    public void update(double timestamp) {
        if (controlState == ControlState.PATHING) {
            control_state = "pathing";
            updatePathing();
        } else if (controlState == ControlState.AUTO_AIM) {
            control_state = "auto aim";
            autoAimController.update();
        } else {
            control_state = "open loop";
        }
        // SmartDashboard.putString("control mode", control_state);
        topRightModule.update(timestamp);
        topLeftModule.update(timestamp);
        bottomLeftModule.update(timestamp);
        bottomRightModule.update(timestamp);

        prevPose = currentPose;

        currentPose = updateOdometry(timestamp);
        fieldPose.setRobotPose(currentPose);

        // This needs to be moved somewhere else.....
        SmartDashboard.putData(fieldPose);

        SmartDashboard.putNumber("X pose", odometry.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Y pose", odometry.getEstimatedPosition().getY());
        SmartDashboard.putNumber("Theta pose", odometry.getEstimatedPosition().getRotation().getDegrees());

        checkThrottleSpeed();

        // if(vision.getRobotPose() != null) {
        // setPose(vision.getRobotPose());
        // }

        // Auto Balance Updates
        double disp = odometry.getEstimatedPosition().getX() - prevX;
        // SmartDashboard.putNumber("disp", disp);
        prevX = odometry.getEstimatedPosition().getX();
        // SmartDashboard.putNumber("prevX", prevX);
        beans += deadband(pigeon.getPitch()) * disp;
        SmartDashboard.putNumber("beans", beans);
    }

    /**
     * Runs components in the submodule that have continuously changing inputs.
     */
    @Override
    public void run() {
        topRightModule.run();
        topLeftModule.run();
        bottomLeftModule.run();
        bottomRightModule.run();
    }

    @Override
    public void stop() {
        controlState = ControlState.OPEN_LOOP;
        topRightModule.stop();
        topLeftModule.stop();
        bottomLeftModule.stop();
        bottomRightModule.stop();
    }

    /**
     * Resets the sensor(s) to zero.
     */
    @Override
    public void zero() {
        if (alliance == Alliance.Blue)
            zeroHeading(180);
        else if (alliance == Alliance.Red)
            zeroHeading(0);
        setPose(new Pose2d(new Translation2d(1.76, 1.477), new Rotation2d(Math.toRadians(pigeon.getAngle()))));

        topRightModule.zero();
        topLeftModule.zero();
        bottomLeftModule.zero();
        bottomRightModule.zero();

    }

    /**
     * Zeroes the heading of the swerve at set Yaw
     */
    public void zeroHeading(double q) {
        pigeon.setYaw(q, Constants.TIMEOUT_MS);
    }

    public void zeroTele(double q) {
        pigeon.setYaw(q, Constants.TIMEOUT_MS);
        setPose(new Pose2d(new Translation2d(1.76, 1.477), new Rotation2d(Math.toRadians(pigeon.getAngle()))));
    }

    public double getYawRate() {
        return pigeon.getRate();
    }

    // public void lockTo90() {
    // if (Math.abs(pigeon.getYaw()-180) > 1 || Math.abs(pigeon.getYaw()-0) > 1)
    // drive(0,0,0.3, false);
    // // pigeon.setYaw(q, Constants.TIMEOUT_MS);
    // // setPose(new Pose2d(new Translation2d(1.76,1.477), new
    // Rotation2d(Math.toRadians(pigeon.getAngle()))));
    // }

    public Field2d getField() {
        return fieldPose;
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                topLeftModule.getModulePosition(),
                topRightModule.getModulePosition(),
                bottomLeftModule.getModulePosition(),
                bottomRightModule.getModulePosition()
        };
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
                topLeftModule.getModuleState(),
                topRightModule.getModuleState(),
                bottomLeftModule.getModuleState(),
                bottomRightModule.getModuleState()
        };
    }

    public static double deadband(double input) {
        if (Math.abs(input) < 0.7) {
            return 0.0;
        }
        return input;
    }

    public double getBeans() {
        return beans;
    }

    public void emptyBucket() {
        beans = 0;
    }

    public double getPitch() {
        return pigeon.getPitch();
    }

    /**
     * Checks the Speed of the throttle and updates the overlimit boolean
     */
    public void checkThrottleSpeed() {
        if (topLeftModule.getThrottlePercentSpeed() > 0.5 || topRightModule.getThrottlePercentSpeed() > 0.3
                || bottomLeftModule.getThrottlePercentSpeed() > 0.5
                || bottomRightModule.getThrottlePercentSpeed() > 0.5)
            overLimit = true;
        else
            overLimit = false;
    }

    /**
     * Checks whether the swerve is over it's safe speed limit
     * 
     * @return is over limit?
     */
    public boolean isOverLimit() {
        // return overLimit;
        return false;
    }

    public void setPose(Pose2d pose) {
        odometry.resetPosition(Rotation2d.fromDegrees(pigeon.getAngle()), getModulePositions(), pose);
    }

    public Pose2d getPose() {
        return odometry.getEstimatedPosition();
    }

    public SwerveDrivePoseEstimator getPoseEstimator() {
        return odometry;
    }

    public synchronized void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        try {
            // visionMeasurementStdDevs = new MatBuilder<N3, N1>(Nat.N3(),
            // Nat.N1()).fill(0.2, 0.2, 0.1);
            // odometry.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
            odometry.addVisionMeasurement(
                    visionRobotPoseMeters,
                    timestampSeconds,
                    visionMeasurementStdDevs);
        } catch (Exception e) {
            System.out.println("Cholesky decomposition failed, reverting...:");
        }
    }

    public Pose2d getPrevPose() {
        return prevPose;
    }

    /**
     * Updates odometry
     * 
     * @param timestamp
     * 
     * @return current position
     */
    private Pose2d updateOdometry(double timestamp) {
        try {
            return odometry.updateWithTime(timestamp,
                    Rotation2d.fromDegrees(pigeon.getAngle()),
                    getModulePositions());
        } catch (Exception e) {
            System.out.println(e);
            return odometry.getEstimatedPosition();
        }
    }

    /**
     * Drives robot (primarily used for teleop manual control)
     * 
     * @param xSpeed        speed in x direction
     * @param ySpeed        speed in y direction
     * @param angularSpeed  turn speed
     * @param fieldOriented
     */
    public void drive(double xSpeed, double ySpeed, double angularSpeed, boolean fieldOriented) {
        if (controlState == ControlState.AUTO_AIM) {
            return;
        }
        controlState = ControlState.OPEN_LOOP;
        boolean ignoreAngle = false;
        var targetState = SwerveConstants.KINEMATICS.toSwerveModuleStates(
                fieldOriented
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                xSpeed, ySpeed, angularSpeed,
                                Rotation2d.fromDegrees(pigeon.getAngle()))
                        : new ChassisSpeeds(xSpeed, ySpeed, angularSpeed));
        SwerveDriveKinematics.desaturateWheelSpeeds(targetState, 1);
        topLeftModule.setTargetState(targetState[0], ignoreAngle, true, true);
        topRightModule.setTargetState(targetState[1], ignoreAngle, true, true);
        bottomLeftModule.setTargetState(targetState[2], ignoreAngle, true, true);
        bottomRightModule.setTargetState(targetState[3], ignoreAngle, true, true);
    }

    public void drive(double xSpeed, double ySpeed, double angularSpeed, boolean fieldOriented, Rotation2d snapAngle) {
        if (Math.abs(angularSpeed) > 0.1) {
            drive(xSpeed, ySpeed, angularSpeed, fieldOriented);
        } else {
            double thetaOutput = snapController.calculate(getPose().getRotation().getRadians(), snapAngle.getRadians());
            drive(xSpeed, ySpeed, thetaOutput, fieldOriented);
        }
    }

    public void drive(double xSpeed, double ySpeed, double angularSpeed, boolean fieldOriented, boolean snap) {
        if (snap && Math.abs(angularSpeed) < 0.1) {
            double thetaOutput = snapController.calculate(getPose().getRotation().getRadians(),
                    DriverStation.getAlliance() == Alliance.Blue ? Math.PI : 0);
            drive(xSpeed, ySpeed, thetaOutput, fieldOriented);
        } else {
            drive(xSpeed, ySpeed, angularSpeed, fieldOriented);
        }
    }

    /**
     * Follow path
     * 
     * @param trajectory desired path
     */
    public void followPath(PathPlannerTrajectory trajectory) {
        if (controlState == ControlState.PATHING) {
            // return;
        }
        if (firstPath) {
            setPose(trajectory.getInitialHolonomicPose());
            firstPath = false;
        }
        controlState = ControlState.PATHING;
        currentTrajectory = trajectory;

        timer.reset();
        timer.start();
    }

    /**
     * A better updatePathing(), featuring:
     * - actually allows the robot to turn
     * - actually reads turn changes from the pathplanner trajectory
     * - fully feedback, no more weird feedforward stuff that doesnt actually work
     */
    private void updatePathing() {
        PathPlannerState state = (PathPlannerState) currentTrajectory.sample(timer.get());
        double xSpeed = xController.calculate(getPose().getX(), state.poseMeters.getX());
        double ySpeed = yController.calculate(getPose().getY(), state.poseMeters.getY());
        double thetaSpeed = thetaController.calculate(getPose().getRotation().getRadians(),
                state.holonomicRotation.getRadians());
        // Math
        // SmartDashboard.putNumber("theta speed", thetaSpeed);

        ChassisSpeeds desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed,
                ySpeed,
                thetaSpeed,
                getPose().getRotation());
        PathPlannerServer.sendPathFollowingData(state.poseMeters, getPose());

        SwerveModuleState[] desiredState = SwerveConstants.KINEMATICS.toSwerveModuleStates(desiredSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredState, 1);
        topLeftModule.setTargetState(desiredState[0], false, true, true);
        topRightModule.setTargetState(desiredState[1], false, true, true);
        bottomLeftModule.setTargetState(desiredState[2], false, true, true);
        bottomRightModule.setTargetState(desiredState[3], false, true, true);
    }

    /**
     * Get total path time
     * 
     * @return path time
     */
    public double getPathingTime() {
        return timer.get();
    }

    /**
     * Check if robot has finished pathing
     * 
     * @return robot pathing state
     */
    public boolean isFinishedPathing() {
        if (xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint()) {
            if (timer.hasElapsed(currentTrajectory.getTotalTimeSeconds())) {
                return true;
            }
        }
        return false;
    }

    public void setOpenLoopSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] desiredState = SwerveConstants.KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredState, 1);
        topLeftModule.setTargetState(desiredState[0], false, true, true);
        topRightModule.setTargetState(desiredState[1], false, true, true);
        bottomLeftModule.setTargetState(desiredState[2], false, true, true);
        bottomRightModule.setTargetState(desiredState[3], false, true, true);
    }

    public ChassisSpeeds getOpenLoopSpeeds() {
        return SwerveConstants.KINEMATICS.toChassisSpeeds(
                new SwerveModuleState(topLeftModule.getThrottlePercentSpeed(),
                        Rotation2d.fromDegrees(topLeftModule.getRotorAngle())),
                new SwerveModuleState(topRightModule.getThrottlePercentSpeed(),
                        Rotation2d.fromDegrees(topRightModule.getRotorAngle())),
                new SwerveModuleState(bottomLeftModule.getThrottlePercentSpeed(),
                        Rotation2d.fromDegrees(bottomLeftModule.getRotorAngle())),
                new SwerveModuleState(bottomRightModule.getThrottlePercentSpeed(),
                        Rotation2d.fromDegrees(bottomRightModule.getRotorAngle())));
    }

    public void setAutoAimLocation(AutoAimLocation location) {
        autoAimController.setTarget(getPose(), location, true);
    }

    public void enableAutoAimController(boolean isEnabled) {
        if (isEnabled) {
            controlState = ControlState.AUTO_AIM;
            autoAimController.enable(isEnabled);
        } else {
            controlState = ControlState.OPEN_LOOP;
        }
    }

    public void setFortniteAutoAimTM(double yDist, double xSpeed) {
        enableAutoAimController(true);
        Rotation2d desiredAngle = new Rotation2d();
        if (DriverStation.getAlliance() == Alliance.Blue) {
            desiredAngle = Rotation2d.fromDegrees(180);
        } else {
            desiredAngle = Rotation2d.fromDegrees(0);
        }
        autoAimController.setTarget(yDist, desiredAngle, xSpeed);
    }

    /**
     * Test swerve modules
     * 
     * @param quadrant       module quadrant [I, II, III, IV]
     * @param throttleOutput output of throttle motor
     * @param rotorOutput    output of rotor motor
     */
    public void testModule(int quadrant, double throttleOutput, double rotorOutput) {
        if (quadrant == 1) {
            topLeftModule.testThrottleAndRotor(throttleOutput, rotorOutput);
        } else if (quadrant == 2) {
            bottomLeftModule.testThrottleAndRotor(throttleOutput, rotorOutput);
        } else if (quadrant == 3) {
            bottomRightModule.testThrottleAndRotor(throttleOutput, rotorOutput);
        } else {
            topRightModule.testThrottleAndRotor(throttleOutput, rotorOutput);
        }
    }

    /**
     * Set rotors in "brake" position
     * 
     * @param enable enable
     */
    public void rotorBrake(boolean enable) {
        if (enable) {
            topRightModule.setRotorAngle(-45);
            topLeftModule.setRotorAngle(-135);
            bottomRightModule.setRotorAngle(225);
            bottomLeftModule.setRotorAngle(315);
        }
    }

    public void addVisionMeasurement(Pose2d plus, double pathingTime, double d) {
    }
}