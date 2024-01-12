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
import raidzero.robot.auto.actions.AutoBalanceAction;
import raidzero.robot.auto.actions.DrivePath;
import raidzero.robot.auto.actions.LambdaAction;
import raidzero.robot.auto.actions.MoveTwoPronged;
import raidzero.robot.auto.actions.ParallelAction;
import raidzero.robot.auto.actions.RunIntakeAction;
import raidzero.robot.auto.actions.SeriesAction;
import raidzero.robot.auto.actions.WaitForEventMarkerAction;
import raidzero.robot.submodules.Swerve;

public class SingleConeSafeClimbSequence extends AutoSequence {
    private static final Swerve mSwerve = Swerve.getInstance();

    private PathPlannerTrajectory mOverRamp = PathPlanner.loadPath("SCC Safe Over",
            SwerveConstants.MAX_DRIVE_VEL_MPS * 0.5,
            SwerveConstants.MAX_DRIVE_ACCEL_MPSPS * 0.5);

    public SingleConeSafeClimbSequence() {
        PathPlannerTrajectory.transformTrajectoryForAlliance(mOverRamp, DriverStation.getAlliance());
    }

    @Override
    public void sequence() {
        PathPlannerTrajectory.transformTrajectoryForAlliance(mOverRamp, DriverStation.getAlliance());
        addAction(
                new SeriesAction(Arrays.asList(
                        new RunIntakeAction(0.1, 0.5),
                        new MoveTwoPronged(ArmConstants.INTER_AUTON_GRID_HIGH,
                                ArmConstants.AUTON_GRID_HIGH, true),
                        new RunIntakeAction(0.5, IntakeConstants.AUTON_CONE_SCORE),

                        new ParallelAction(Arrays.asList(
                                new AsyncArmHomeAction(),
                                new DrivePath(mOverRamp))),

                        // Balance
                        new AutoBalanceAction(false, SwerveConstants.AUTO_BEANS),
                        new LambdaAction(() -> mSwerve.rotorBrake(true)))));
    }

    @Override
    public void onEnded() {
    }

    @Override
    public String getName() {
        return "Single Cone Safe Climb Sequence";
    }
}
