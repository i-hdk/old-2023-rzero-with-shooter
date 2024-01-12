package raidzero.robot.auto.sequences;

import java.util.Arrays;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

import raidzero.robot.Constants.ArmConstants;
import raidzero.robot.Constants.IntakeConstants;
import raidzero.robot.Constants.SwerveConstants;
import raidzero.robot.auto.actions.ArmHomeAction;
import raidzero.robot.auto.actions.AsyncArmHomeAction;
import raidzero.robot.auto.actions.AsyncDrivePath;
import raidzero.robot.auto.actions.AsyncRunIntakeAction;
import raidzero.robot.auto.actions.DrivePath;
import raidzero.robot.auto.actions.LambdaAction;
import raidzero.robot.auto.actions.MoveTwoPronged;
import raidzero.robot.auto.actions.OTFdrivePath;
import raidzero.robot.auto.actions.ParallelAction;
import raidzero.robot.auto.actions.RunIntakeAction;
import raidzero.robot.auto.actions.SeriesAction;
import raidzero.robot.auto.actions.CubeAlighmentAction;
import raidzero.robot.auto.actions.WaitAction;
import raidzero.robot.auto.actions.WaitForEventMarkerAction;
import raidzero.robot.auto.actions.WaitForFlyingCube;
import raidzero.robot.submodules.Swerve;
import raidzero.robot.submodules.Vision;

public class FlyingCubeSequence extends AutoSequence {
    private static final Swerve mSwerve = Swerve.getInstance();
    private static final Vision mVision = Vision.getInstance();

    private PathPlannerTrajectory mOut = PathPlanner.loadPath("Flying Cube Out",
            SwerveConstants.MAX_DRIVE_VEL_MPS * 0.5,
            SwerveConstants.MAX_DRIVE_ACCEL_MPSPS * 0.5);
    private PathPlannerTrajectory mReturn = PathPlanner.loadPath("Flying Cube Return",
            SwerveConstants.MAX_DRIVE_VEL_MPS * 0.5,
            SwerveConstants.MAX_DRIVE_ACCEL_MPSPS * 0.5);

    public FlyingCubeSequence() {
        PathPlannerTrajectory.transformTrajectoryForAlliance(mOut, DriverStation.getAlliance());
        PathPlannerTrajectory.transformTrajectoryForAlliance(mReturn, DriverStation.getAlliance());
    }

    @Override
    public void sequence() {
        addAction(
                new SeriesAction(Arrays.asList(
                        new ParallelAction(Arrays.asList(
                                new DrivePath(mOut),
                                new SeriesAction(Arrays.asList(
                                        new AsyncRunIntakeAction(IntakeConstants.AUTON_CUBE_INTAKE),
                                        new MoveTwoPronged(
                                                ArmConstants.INTER_REV_CUBE_FLOOR_INTAKE,
                                                ArmConstants.REV_CUBE_FLOOR_INTAKE, false),
                                        new WaitForFlyingCube(),
                                        // new WaitForEventMarkerAction(mOut, "fIntake", mSwerve.getPathingTime()),
                                        new CubeAlighmentAction(mOut.getEndState().poseMeters))),
                        // Return to community
                        new ParallelAction(Arrays.asList(
                                new AsyncArmHomeAction(),
                                new RunIntakeAction(1.0, -0.3)))
                )))));
    }

    @Override
    public void onEnded() {
    }

    @Override
    public String getName() {
        return "Flying Cube Sequence";
    }
}

// new OTFdrivePath(1,
// new PathConstraints(SwerveConstants.MAX_DRIVE_VEL_MPS * 0.5,
// SwerveConstants.MAX_DRIVE_ACCEL_MPSPS * 0.5),
// new PathPoint(
// mOut.sample(mSwerve.getPathingTime() + 1).poseMeters
// .getTranslation(),
// mOut.sample(mSwerve.getPathingTime() + 1).poseMeters
// .getRotation(),
// Rotation2d.fromDegrees(180), 1),
// new PathPoint(
// mOut.getEndState().poseMeters.getTranslation(),
// // mOut.sample(mSwerve.getPathingTime()).poseMeters
// // .getTranslation()
// // .plus(new Translation2d(Math.abs(mVision.getCubeY()),
// // Math.abs(mVision.getCubeX()))),
// mOut.getEndState().poseMeters.getRotation()
// .minus(Rotation2d.fromDegrees(mVision.getCubeAngle())),
// Rotation2d.fromDegrees(100),
// 1))