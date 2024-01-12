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

public class ConeCubeBumpClimbSequenceBlue extends AutoSequence {
    private static final Swerve mSwerve = Swerve.getInstance();

    private PathPlannerTrajectory mOut = PathPlanner.loadPath("CC Bump Pickup Blue", SwerveConstants.MAX_DRIVE_VEL_MPS * 0.7,
            SwerveConstants.MAX_DRIVE_ACCEL_MPSPS * 0.7);
    private PathPlannerTrajectory mReturn = PathPlanner.loadPath("CC Bump Score Blue", SwerveConstants.MAX_DRIVE_VEL_MPS * 1.0,
            SwerveConstants.MAX_DRIVE_ACCEL_MPSPS * 1.0);
    private PathPlannerTrajectory mBalance = PathPlanner.loadPath("CC Bump Balance Blue",
            SwerveConstants.MAX_DRIVE_VEL_MPS * 1.0,
            SwerveConstants.MAX_DRIVE_ACCEL_MPSPS * 1.0);

    public ConeCubeBumpClimbSequenceBlue() {
        PathPlannerTrajectory.transformTrajectoryForAlliance(mOut, DriverStation.getAlliance());
        PathPlannerTrajectory.transformTrajectoryForAlliance(mReturn, DriverStation.getAlliance());
        PathPlannerTrajectory.transformTrajectoryForAlliance(mBalance, DriverStation.getAlliance());
    }

    @Override
    public void sequence() {
        addAction(
                new SeriesAction(Arrays.asList(
                        // Score Cone
                        new RunIntakeAction(0.1, 0.5),
                        new MoveTwoPronged(ArmConstants.INTER_AUTON_GRID_HIGH,
                                ArmConstants.AUTON_GRID_HIGH, true),
                        new RunIntakeAction(0.5, IntakeConstants.AUTON_CONE_SCORE),

                        // Go To Cube + Scoop
                        new ParallelAction(Arrays.asList(
                                new AsyncArmHomeAction(),
                                new DrivePath(mOut),
                                new SeriesAction(Arrays.asList(
                                        new WaitForEventMarkerAction(mOut, "fIntake",
                                                mSwerve.getPathingTime()),
                                        new MoveTwoPronged(
                                                ArmConstants.INTER_REV_CUBE_FLOOR_INTAKE,
                                                ArmConstants.REV_CUBE_FLOOR_INTAKE, false))),
                                new AsyncRunIntakeAction(IntakeConstants.AUTON_CUBE_INTAKE))),

                        // Return to community
                        new ParallelAction(Arrays.asList(
                                new AsyncArmHomeAction(),
                                new DrivePath(mReturn),
                                new SeriesAction(Arrays.asList(
                                        new WaitAction(1.2),
                                        new MoveTwoPronged(ArmConstants.INTER_CUBE_GRID_HIGH,
                                                ArmConstants.CUBE_GRID_HIGH, true))),
                                new RunIntakeAction(1.0, -0.3))),

                        // Score Cube
                        new RunIntakeAction(0.5, IntakeConstants.AUTON_CUBE_SCORE),

                        new ParallelAction(Arrays.asList(
                                new ArmHomeAction(),
                                new DrivePath(mBalance))),
                        new AutoBalanceAction(true, SwerveConstants.AUTO_BEANS),
                        new LambdaAction(() -> mSwerve.rotorBrake(true))

                )));
    }

    @Override
    public void onEnded() {
    }

    @Override
    public String getName() {
        return "Cone Cube Bump Climb Sequence Blue";
    }
}
