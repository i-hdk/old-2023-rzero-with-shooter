package raidzero.robot.utils;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import raidzero.robot.Constants.SwerveConstants;
import raidzero.robot.Constants.VisionConstants;
import raidzero.robot.submodules.Swerve;

public class AutoAimController {
    public enum AutoAimLocation {
        LL, LM, LR, ML, MM, MR, RL, RM, RR,
        R_LOAD, L_LOAD
    };

    public enum LoadStationLocation { 
        WALL_SIDE, GRID_SIDE
    }

    private PIDController mXController, mYController;
    private ProfiledPIDController mThetaController, mFortniteAimAssistYController;
    private TrajectoryConfig mTrajectoryConfig;
    private Pose2d mPoseError = new Pose2d();
    private Rotation2d mRotationError = new Rotation2d();
    private Timer mTimer = new Timer();

    private Trajectory mTrajectory;
    private Rotation2d mEndHeading;

    private double mDesiredYPose;
    private Rotation2d mDesiredRotationPose = new Rotation2d();
    private double mDesiredXSpeed;
    private boolean mUsingTrajectory = false; 

    private boolean firstStart = true;

    private Field2d field;

    private boolean mEnabled = false;

    private static final Swerve mSwerve = Swerve.getInstance();

    /**
     * Create new Auto Aim Controller
     * 
     * @param xController     x PID controller (meters)
     * @param yController     y PID controller (meters)
     * @param thetaController theta PID controller (radians)
     */
    public AutoAimController(
            PIDController xController,
            PIDController yController,
            ProfiledPIDController thetaController,
            TrajectoryConfig trajectoryConfig) {
        mXController = xController;
        mYController = yController;
        mThetaController = thetaController;
        mTrajectoryConfig = trajectoryConfig;

        mThetaController.enableContinuousInput(-Math.PI, Math.PI);
        mThetaController.reset(mSwerve.getPose().getRotation().getRadians(), 0);

        mFortniteAimAssistYController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(3, 3));
        // mFortniteAimAssistYController.reset(0, 0);

        field = mSwerve.getField();

        mEndHeading = new Rotation2d();
    }

    // public Field2d getField() {
    // return field;
    // }

    /**
     * Set target points (quintic hermite spline)
     * 
     * @param points     points along the path
     * @param endHeading end heading of chassis
     */
    public void setTarget(List<Pose2d> points, Rotation2d endHeading) {
        mUsingTrajectory = true;
        Trajectory traj = TrajectoryGenerator.generateTrajectory(
                points,
                mTrajectoryConfig);

        field.getObject("traj").setTrajectory(traj);

        followPath(traj, endHeading);
    }

    /**
     * Set target points (cubic hermite spline)
     * 
     * @param startPose   starting position
     * @param interPoints intermediate position
     * @param endPose     end position
     * @param endHeading  end heading
     */
    public void setTarget(Pose2d startPose, List<Translation2d> interPoints, Pose2d endPose, Rotation2d endHeading) {
        mUsingTrajectory = true;
        Trajectory traj = TrajectoryGenerator.generateTrajectory(
                startPose,
                interPoints,
                endPose,
                mTrajectoryConfig);
        field.getObject("traj").setTrajectory(traj);
        followPath(traj, endHeading);
    }

    /**
     * Set target location
     * 
     * @param startPose starting position
     * @param location  desired auto aim location
     */
    public void setTarget(Pose2d startPose, AutoAimLocation location) {
        Alliance alliance = DriverStation.getAlliance();

        double fieldLengthMeters = 16.5;
        double blueLongThreshold = 3;
        double blueUltraThreshold = 5.5;
        double blueLeftThreshold = 3;
        double blueScoringX = 1.85;
        double redScoringXOffset = -0.03;
        Translation2d blueLeftLongInterPoint = new Translation2d(2.6, 4.5);
        Translation2d blueRightLongInterPoint = new Translation2d(2.6, 1.35);
        Translation2d blueLeftUltraInterPoint = new Translation2d(4.5, 4.7);
        Translation2d blueRightUltraInterPoint = new Translation2d(4.5, 1.15);

        Rotation2d defaultRotation = alliance == Alliance.Blue ? Rotation2d.fromDegrees(180)
                : Rotation2d.fromDegrees(0);

        boolean longPath = startPose.getX() > blueLongThreshold;
        boolean ultraLongPath = startPose.getX() > blueUltraThreshold;
        boolean left = startPose.getY() > blueLeftThreshold;
        double scoringX = blueScoringX;

        Translation2d leftLongInterPoint = blueLeftLongInterPoint;
        Translation2d rightLongInterPoint = blueRightLongInterPoint;
        Translation2d leftUltraInterPoint = blueLeftUltraInterPoint;
        Translation2d rightUltraInterPoint = blueRightUltraInterPoint;
        if (alliance == Alliance.Red) {
            longPath = startPose.getX() < fieldLengthMeters - blueLongThreshold;
            ultraLongPath = startPose.getX() < fieldLengthMeters - blueUltraThreshold;
            left = startPose.getY() < blueLeftThreshold;

            leftLongInterPoint = new Translation2d(fieldLengthMeters - blueLeftLongInterPoint.getX(),
                    blueRightLongInterPoint.getY());
            rightLongInterPoint = new Translation2d(fieldLengthMeters - blueLeftLongInterPoint.getX(),
                    blueLeftLongInterPoint.getY());
            leftUltraInterPoint = new Translation2d(fieldLengthMeters - blueLeftUltraInterPoint.getX(),
                    blueRightUltraInterPoint.getY());
            rightUltraInterPoint = new Translation2d(fieldLengthMeters - blueLeftUltraInterPoint.getX(),
                    blueLeftUltraInterPoint.getY());

            scoringX = fieldLengthMeters - blueScoringX + redScoringXOffset;
        }

        Pose2d start, end;
        List<Translation2d> interPoints = new ArrayList<Translation2d>();

        start = new Pose2d(startPose.getX(), startPose.getY(), defaultRotation);
        end = new Pose2d(scoringX, getYOfAutoAimLocation(location), defaultRotation);

        // maybe check
        if (!longPath) {
            start = new Pose2d(start.getX(), start.getY(),
                    new Rotation2d(end.getX() - start.getX(), end.getY() - start.getY()));
        }

        if (ultraLongPath) {
            if (left) {
                interPoints.add(leftUltraInterPoint);
                start = new Pose2d(start.getX(), start.getY(), new Rotation2d(leftUltraInterPoint.getX() - start.getX(),
                        leftUltraInterPoint.getY() - start.getY()));
            } else {
                interPoints.add(rightUltraInterPoint);
                start = new Pose2d(start.getX(), start.getY(), new Rotation2d(
                        rightUltraInterPoint.getX() - start.getX(), rightUltraInterPoint.getY() - start.getY()));
            }
        }

        if (longPath) {
            if (left) {
                interPoints.add(leftLongInterPoint);
            } else {
                interPoints.add(rightLongInterPoint);
            }
        }

        Translation2d secondToLastPoint;
        if (interPoints.size() > 0) {
            secondToLastPoint = interPoints.get(0);
        } else {
            secondToLastPoint = new Translation2d(start.getX(), start.getY());
        }

        end = new Pose2d(end.getX(), end.getY(),
                new Rotation2d(end.getX() - secondToLastPoint.getX(), end.getY() - secondToLastPoint.getY()));

        int rotationMult = alliance == Alliance.Blue ? 1 : -1;
        if (secondToLastPoint.getY() > end.getY()) {
            end = new Pose2d(end.getX(), end.getY(),
                    Rotation2d.fromDegrees(end.getRotation().getDegrees() - (30 * rotationMult)));
        } else if (secondToLastPoint.getY() < end.getY()) {
            end = new Pose2d(end.getX(), end.getY(),
                    Rotation2d.fromDegrees(end.getRotation().getDegrees() + (30 * rotationMult)));
        }

        //Rotation2d endHeading = alliance == Alliance.Blue ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0);

        setTarget(start, interPoints, end, getAutoAimHeading(location));
    }

    /**
     * Set target location
     * 
     * @param startPose          start pose
     * @param location           desired end location
     * @param accountForStartVel account for starting velocity
     */
    public void setTarget(Pose2d startPose, AutoAimLocation location, boolean accountForStartVel) {
        if (accountForStartVel) {
            ChassisSpeeds speeds = SwerveConstants.KINEMATICS.toChassisSpeeds(mSwerve.getModuleStates());
            double speedMPS = Math.abs(Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
            mTrajectoryConfig.setStartVelocity(speedMPS);
        }
        setTarget(startPose, location);
    }

    public void setTarget(double desiredYMeters, Rotation2d desiredTheta, double xSpeed) {
        mUsingTrajectory = false;
        mDesiredYPose = desiredYMeters;
        mDesiredRotationPose = desiredTheta;
        mDesiredXSpeed = xSpeed;
    }

    /**
     * Set enabled state of controller
     * 
     * @param enabled enabled
     */
    public void enable(boolean enabled) {
        mEnabled = enabled;
    }

    /** Update Controller */
    public void update() {
        if (!mEnabled)
            return;
        if(mUsingTrajectory) {
            Trajectory.State currState = mTrajectory.sample(mTimer.get());
            // field.setRobotPose(currState.poseMeters.getX(), currState.poseMeters.getY(),
            // mEndHeading);
            // SmartDashboard.putData(field);

            ChassisSpeeds speeds = calculate(mSwerve.getPose(), currState, mEndHeading);
            mSwerve.setOpenLoopSpeeds(speeds);
        } else {
            if(firstStart) {
                mFortniteAimAssistYController.reset(mSwerve.getPose().getY(), 0);
                mThetaController.reset(mSwerve.getPose().getRotation().getRadians(), 0);
            }
            firstStart = false;
            ChassisSpeeds speeds = new ChassisSpeeds(
                mDesiredXSpeed, 
                mFortniteAimAssistYController.calculate(mSwerve.getPose().getY(), mDesiredYPose), 
                mThetaController.calculate(mSwerve.getPose().getRotation().getRadians(), mDesiredRotationPose.getRadians())
            );
            mSwerve.setOpenLoopSpeeds(speeds);
        }
    }

    /**
     * Get positional error
     * 
     * @return positional error (meters)
     */
    public Pose2d getError() {
        return new Pose2d(mPoseError.getX(), mPoseError.getY(), mRotationError);
    }

    /**
     * Set tolerance
     * 
     * @param tolerance tolerance (meters & radians)
     */
    public void setTolerance(Pose2d tolerance) {
        mXController.setTolerance(tolerance.getX());
        mYController.setTolerance(tolerance.getY());
        mThetaController.setTolerance(tolerance.getRotation().getRadians());
    }

    /**
     * Check if robot is at target
     * 
     * @return robot at target
     */
    public boolean atTarget() {
        return mXController.atSetpoint() && mYController.atSetpoint() && mThetaController.atSetpoint();
    }

    /**
     * Calculate output speeds
     * 
     * @param currPose       current robot pose (meters)
     * @param trajState      desired trajectory state (meters)
     * @param desiredHeading desired final heading
     * @return output chassis speeds
     */
    private ChassisSpeeds calculate(Pose2d currPose, Trajectory.State trajState, Rotation2d desiredHeading) {
        // double xFF = trajState.velocityMetersPerSecond *
        // trajState.poseMeters.getRotation().getCos();
        // double yFF = trajState.velocityMetersPerSecond *
        // trajState.poseMeters.getRotation().getSin();
        double xFF = 0.0;
        double yFF = 0.0;
        double thetaFF = mThetaController.calculate(currPose.getRotation().getRadians());

        SmartDashboard.putNumber("desired theta setpoint", mThetaController.getSetpoint().position);
        SmartDashboard.putNumber("desired heading", desiredHeading.getRadians());

        mPoseError = trajState.poseMeters.relativeTo(currPose);
        mRotationError = desiredHeading.minus(currPose.getRotation());

        double xFeedback = mXController.calculate(currPose.getX(), trajState.poseMeters.getX());
        double yFeedback = mYController.calculate(currPose.getY(), trajState.poseMeters.getY());

        // Return next output.%
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                xFF + xFeedback,
                yFF + yFeedback,
                thetaFF,
                currPose.getRotation());
    }

    /**
     * Follow Trajectory
     * 
     * @param trajectory desired trajectory
     * @param endHeading desired end heading
     */
    private void followPath(Trajectory trajectory, Rotation2d endHeading) {
        mTimer.reset();
        mTimer.start();
        mTrajectory = trajectory;
        mEndHeading = endHeading;
        mThetaController.setGoal(new TrapezoidProfile.State(mEndHeading.getRadians(), 0));
        System.out.println(endHeading.getRadians());
        System.out.println(mThetaController.getGoal().position);
        System.out.println(mThetaController.calculate(mSwerve.getPose().getRotation().getRadians()));
        System.out.println(mThetaController.getPositionError());
        System.out.println(mSwerve.getPose().getRotation().getRadians());
        System.out.println();
    }

    private double getYOfAutoAimLocation(AutoAimLocation location) {
        Alliance alliance = DriverStation.getAlliance();
        switch (location) {
            case LL:
                return alliance == Alliance.Blue ? VisionConstants.BLL : VisionConstants.RLL;
            case LM:
                return alliance == Alliance.Blue ? VisionConstants.BLM : VisionConstants.RLM;
            case LR:
                return alliance == Alliance.Blue ? VisionConstants.BLR : VisionConstants.RLR;
            case ML:
                return alliance == Alliance.Blue ? VisionConstants.BML : VisionConstants.RML;
            case MM:
                return alliance == Alliance.Blue ? VisionConstants.BMM : VisionConstants.RMM;
            case MR:
                return alliance == Alliance.Blue ? VisionConstants.BMR : VisionConstants.RMR;
            case RL:
                return alliance == Alliance.Blue ? VisionConstants.BRL : VisionConstants.RRL;
            case RM:
                return alliance == Alliance.Blue ? VisionConstants.BRM : VisionConstants.RRM;
            case RR:
                return alliance == Alliance.Blue ? VisionConstants.BRR : VisionConstants.RRR;
            default:
                return 0;
        }
    }

    private Rotation2d getAutoAimHeading (AutoAimLocation location) {
        Alliance alliance = DriverStation.getAlliance();
        switch (location) {
            case LL:
                return alliance == Alliance.Blue ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(-22);
            case RR:
                return alliance == Alliance.Blue ? Rotation2d.fromDegrees(155) : Rotation2d.fromDegrees(0);
            default:
                return alliance == Alliance.Blue ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0);
        }
    }
}