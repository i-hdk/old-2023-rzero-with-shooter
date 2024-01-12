package raidzero.robot.auto.sequences;

import java.util.Arrays;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;

import raidzero.robot.Constants.ArmConstants;
import raidzero.robot.Constants.IntakeConstants;
import raidzero.robot.Constants.SwerveConstants;
import raidzero.robot.auto.actions.ArmHomeAction;
import raidzero.robot.auto.actions.AsyncArmHomeAction;
import raidzero.robot.auto.actions.AsyncRunIntakeAction;
import raidzero.robot.auto.actions.AutoBalanceAction;
import raidzero.robot.auto.actions.DrivePath;
import raidzero.robot.auto.actions.LambdaAction;
import raidzero.robot.auto.actions.MoveTwoPronged;
import raidzero.robot.auto.actions.ParallelAction;
import raidzero.robot.auto.actions.RunIntakeAction;
import raidzero.robot.auto.actions.SeriesAction;
import raidzero.robot.auto.actions.WaitForEventMarkerAction;
import raidzero.robot.submodules.Swerve;

public class CenterConeCubeClimbSequence extends AutoSequence {
    private static final Swerve mSwerve = Swerve.getInstance();

    private PathPlannerTrajectory mOverRamp = PathPlanner.loadPath("SCC Over", SwerveConstants.MAX_DRIVE_VEL_MPS * 0.5,
            SwerveConstants.MAX_DRIVE_ACCEL_MPSPS * 0.5);
    private PathPlannerTrajectory mReturn = PathPlanner.loadPath("Center Cube Score",
            SwerveConstants.MAX_DRIVE_VEL_MPS * 0.5,
            SwerveConstants.MAX_DRIVE_ACCEL_MPSPS * 0.5);

    public CenterConeCubeClimbSequence() {
        PathPlannerTrajectory.transformTrajectoryForAlliance(mOverRamp, DriverStation.getAlliance());
        PathPlannerTrajectory.transformTrajectoryForAlliance(mReturn, DriverStation.getAlliance());
    }

    @Override
    public void sequence() {
        PathPlannerTrajectory.transformTrajectoryForAlliance(mOverRamp, DriverStation.getAlliance());
        PathPlannerTrajectory.transformTrajectoryForAlliance(mReturn, DriverStation.getAlliance());
        addAction(
                new SeriesAction(Arrays.asList(
                        new RunIntakeAction(0.1, 0.5),
                        new MoveTwoPronged(ArmConstants.INTER_AUTON_GRID_HIGH,
                                ArmConstants.AUTON_GRID_HIGH, true),
                        new RunIntakeAction(0.5, IntakeConstants.AUTON_CONE_SCORE),

                        // Get Cube
                        new ParallelAction(Arrays.asList(
                                new AsyncArmHomeAction(),
                                new DrivePath(mOverRamp),
                                new SeriesAction(Arrays.asList(
                                        new WaitForEventMarkerAction(mOverRamp, "fIntake",
                                                mSwerve.getPathingTime()),
                                        new MoveTwoPronged(
                                                ArmConstants.INTER_REV_CUBE_FLOOR_INTAKE,
                                                ArmConstants.REV_CUBE_FLOOR_INTAKE, false))),
                                new AsyncRunIntakeAction(IntakeConstants.AUTON_CUBE_INTAKE))),

                        // Return to community
                        new ParallelAction(Arrays.asList(
                                new AsyncArmHomeAction(),
                                new RunIntakeAction(1.0, -0.3),
                                new SeriesAction(Arrays.asList(
                                        new DrivePath(mReturn),
                                        // Score Cube
                                        new RunIntakeAction(0.5, IntakeConstants.AUTON_CUBE_SCORE))),
                                new SeriesAction(Arrays.asList(
                                        new WaitForEventMarkerAction(mReturn, "cScore",
                                                mSwerve.getPathingTime()),
                                        new MoveTwoPronged(ArmConstants.INTER_CUBE_GRID_HIGH,
                                                ArmConstants.CUBE_GRID_HIGH, true))))),
                        // Balance
                        new ParallelAction(Arrays.asList(
                                new AsyncArmHomeAction(),
                                new RunIntakeAction(3.0, -0.3),
                                new AutoBalanceAction(true, SwerveConstants.AUTO_BEANS))),

                        new LambdaAction(() -> mSwerve.rotorBrake(true)))));
    }

    @Override
    public void onEnded() {
    }

    @Override
    public String getName() {
        return "Center Cone Cube Climb Sequence";
    }
}
