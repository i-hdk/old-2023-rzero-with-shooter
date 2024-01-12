package raidzero.robot.auto.sequences;

import java.util.Arrays;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import raidzero.robot.Constants.SwerveConstants;
import raidzero.robot.auto.actions.AutoBalanceAction;
import raidzero.robot.auto.actions.DrivePath;
import raidzero.robot.auto.actions.LambdaAction;
import raidzero.robot.auto.actions.ParallelAction;
import raidzero.robot.auto.actions.SeriesAction;
import raidzero.robot.auto.actions.WaitForEventMarkerAction;
import raidzero.robot.auto.actions.RunIntakeAction;

import raidzero.robot.submodules.Intake;
import raidzero.robot.submodules.Swerve;

public class TestSequence extends AutoSequence {
    private PathPlannerTrajectory mTrajectory = PathPlanner.loadPath("TestPath", SwerveConstants.MAX_DRIVE_VEL_MPS,
            SwerveConstants.MAX_DRIVE_ACCEL_MPSPS);
    private Swerve mSwerve = Swerve.getInstance();
    private Intake mIntake = Intake.getInstance();

    @Override
    public void sequence() {
        // addAction(
        // new ParallelAction(Arrays.asList(
        // new DrivePath(mTrajectory),
        // new SeriesAction(Arrays.asList(
        // new WaitForEventMarkerAction(mTrajectory, "deez nuts",
        // mSwerve.getPathingTime()),
        // new LambdaAction(() -> mIntake.set(0.5)))
        // ))
        // )
        // );
        addAction(
            new SeriesAction(Arrays.asList(
                new AutoBalanceAction(false, SwerveConstants.AUTO_BEANS),
                new LambdaAction(() -> mSwerve.rotorBrake(true)
            )))
                //new DrivePath(mTrajectory)
                
                
        // new RunIntakeAction(5, 0.5)
        );
    }

    @Override
    public void onEnded() {
        System.out.println("TestSequence ended!");
    }

    @Override
    public String getName() {
        return "Test Sequence";
    }
}
