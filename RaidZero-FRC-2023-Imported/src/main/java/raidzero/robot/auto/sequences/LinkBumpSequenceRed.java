package raidzero.robot.auto.sequences;

import java.util.Arrays;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import raidzero.robot.Constants.ArmConstants;
import raidzero.robot.Constants.IntakeConstants;
import raidzero.robot.Constants.SwerveConstants;
import raidzero.robot.auto.actions.ArmHomeAction;
import raidzero.robot.auto.actions.AsyncArmHomeAction;
import raidzero.robot.auto.actions.AsyncDrivePath;
import raidzero.robot.auto.actions.AsyncMoveTwoPronged;
import raidzero.robot.auto.actions.AsyncRunIntakeAction;
import raidzero.robot.auto.actions.AutoBalanceAction;
import raidzero.robot.auto.actions.DrivePath;
import raidzero.robot.auto.actions.LambdaAction;
import raidzero.robot.auto.actions.MoveThreePronged;
import raidzero.robot.auto.actions.MoveTwoPronged;
import raidzero.robot.auto.actions.ParallelAction;
import raidzero.robot.auto.actions.RunIntakeAction;
import raidzero.robot.auto.actions.SeriesAction;
import raidzero.robot.auto.actions.WaitAction;
import raidzero.robot.auto.actions.WaitForEventMarkerAction;
import raidzero.robot.submodules.Swerve;

public class LinkBumpSequenceRed extends AutoSequence {
    private static final Swerve mSwerve = Swerve.getInstance();

    private PathPlannerTrajectory mTurn = PathPlanner.loadPath("CC Bump Turn Red",
            SwerveConstants.MAX_DRIVE_VEL_MPS * 0.3,
            SwerveConstants.MAX_DRIVE_ACCEL_MPSPS * 0.3);
    private PathPlannerTrajectory mFirstPickup = PathPlanner.loadPath("Link Bump First Pickup Red",
            SwerveConstants.MAX_DRIVE_VEL_MPS * 0.8,
            SwerveConstants.MAX_DRIVE_ACCEL_MPSPS * 1.0);
    private PathPlannerTrajectory mFirstScore = PathPlanner.loadPath("Link Bump First Score Red",
            SwerveConstants.MAX_DRIVE_VEL_MPS * 1.5,
            SwerveConstants.MAX_DRIVE_ACCEL_MPSPS * 1.0);
    private PathPlannerTrajectory mSecondPickup = PathPlanner.loadPath("Link Bump Second Pickup Red",
            SwerveConstants.MAX_DRIVE_VEL_MPS * 1.5,
            SwerveConstants.MAX_DRIVE_ACCEL_MPSPS * 0.8);
    private PathPlannerTrajectory mSecondScore = PathPlanner.loadPath("Link Bump Second Score Red",
            SwerveConstants.MAX_DRIVE_VEL_MPS * 1.5,
            SwerveConstants.MAX_DRIVE_ACCEL_MPSPS * 1.0);

    public LinkBumpSequenceRed() {
        PathPlannerTrajectory.transformTrajectoryForAlliance(mTurn, DriverStation.getAlliance());
        PathPlannerTrajectory.transformTrajectoryForAlliance(mFirstPickup, DriverStation.getAlliance());
        PathPlannerTrajectory.transformTrajectoryForAlliance(mFirstScore, DriverStation.getAlliance());
        PathPlannerTrajectory.transformTrajectoryForAlliance(mSecondPickup, DriverStation.getAlliance());
        PathPlannerTrajectory.transformTrajectoryForAlliance(mSecondScore, DriverStation.getAlliance());
    }

    @Override
    public void sequence() {
        addAction(
                new SeriesAction(Arrays.asList(
                        // Score Cone
                        new ParallelAction(Arrays.asList(
                                new RunIntakeAction(0.1, 0.5),
                                new AsyncDrivePath(mTurn),
                                new MoveTwoPronged(ArmConstants.INTER_AUTON_EXTENDED_GRID_HIGH,
                                        ArmConstants.AUTON_EXTENDED_GRID_HIGH, true))),
                        new RunIntakeAction(0.25, IntakeConstants.AUTON_CONE_SCORE),

                        // Go To Cube + Scoop
                        new ParallelAction(Arrays.asList(
                                new AsyncArmHomeAction(),
                                new DrivePath(mFirstPickup),
                                new SeriesAction(Arrays.asList(
                                        new WaitForEventMarkerAction(mFirstPickup, "fIntake",
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
                                        new DrivePath(mFirstScore),
                                        // Score Cube
                                        new RunIntakeAction(0.5, IntakeConstants.AUTON_CUBE_SCORE))),
                                new SeriesAction(Arrays.asList(
                                        new WaitForEventMarkerAction(mFirstScore, "cScore",
                                                mSwerve.getPathingTime()),
                                        new MoveTwoPronged(ArmConstants.INTER_CUBE_GRID_HIGH,
                                                ArmConstants.CUBE_GRID_HIGH, true))))),

                        // Go To Second Cube + Scoop
                        new ParallelAction(Arrays.asList(
                                new AsyncArmHomeAction(),
                                new RunIntakeAction(0.5, IntakeConstants.AUTON_CUBE_INTAKE),
                                new DrivePath(mSecondPickup),
                                new SeriesAction(Arrays.asList(
                                        new WaitForEventMarkerAction(mSecondPickup, "fIntake",
                                                mSwerve.getPathingTime()),
                                        new AsyncRunIntakeAction(IntakeConstants.AUTON_CUBE_INTAKE),
                                        new MoveTwoPronged(
                                                ArmConstants.INTER_REV_CUBE_FLOOR_INTAKE,
                                                ArmConstants.REV_CUBE_FLOOR_INTAKE, false))))),

                        // Return to community
                        new ParallelAction(Arrays.asList(
                                new AsyncArmHomeAction(),
                                new RunIntakeAction(0.7, -0.3),
                                new SeriesAction(Arrays.asList(
                                        new DrivePath(mSecondScore),
                                        // Score Cube
                                        new RunIntakeAction(0.5, IntakeConstants.AUTON_CUBE_SCORE),
                                        new ArmHomeAction())),
                                new SeriesAction(Arrays.asList(
                                        new WaitForEventMarkerAction(mSecondScore, "cScore",
                                                mSwerve.getPathingTime()),
                                        new AsyncMoveTwoPronged(ArmConstants.INTER_CUBE_GRID_MEDIUM,
                                                ArmConstants.CUBE_GRID_MEDIUM, true)))))

                )));
    }

    @Override
    public void onEnded() {
    }

    @Override
    public String getName() {
        return "Link Bump Sequence Red";
    }
}
