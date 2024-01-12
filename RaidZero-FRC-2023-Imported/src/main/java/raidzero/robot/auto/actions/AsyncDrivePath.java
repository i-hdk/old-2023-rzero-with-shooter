package raidzero.robot.auto.actions;

import com.pathplanner.lib.path.PathPlannerTrajectory;

//import com.pathplanner.lib.PathPlannerTrajectory;

import raidzero.robot.submodules.Swerve;

/**
 * Action for following a path.
 */
public class AsyncDrivePath implements Action {

    private static final Swerve swerve = Swerve.getInstance();

    private PathPlannerTrajectory mTrajectory;

    public AsyncDrivePath(PathPlannerTrajectory trajectory) {
        mTrajectory = trajectory;
    }

    @Override
    public boolean isFinished() {
        // if (swerve.isFinishedPathing()) {
        //     System.out.println("[Auto] Path finished!");
        //     return true;
        // }
        return true;
    }

    @Override
    public void start() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");
        swerve.followPath(mTrajectory);
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
        swerve.stop();
    }
}
