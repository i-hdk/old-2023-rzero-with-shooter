package raidzero.robot.auto.actions;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.wpilibj.Timer;
import raidzero.robot.submodules.Swerve;

/**
 * Action for following a path.
 */
public class OTFdrivePath implements Action {

    private static final Swerve swerve = Swerve.getInstance();
    private Timer timer = new Timer();
    private double duration;
    private PathPlannerTrajectory mTrajectory;
    private boolean isRunning = false;

    public OTFdrivePath(double delay, PathConstraints constraints, PathPoint point1, PathPoint point2,
            PathPoint... points) {
        mTrajectory = PathPlanner.generatePath(constraints, false, point1, point2, points);
        this.duration = delay;
    }

    @Override
    public boolean isFinished() {
        if (swerve.isFinishedPathing()) {
            System.out.println("[Auto] Path finished!");
            return true;
        }
        return false;
    }

    @Override
    public void start() {
        timer.reset();
        timer.start();
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");

    }

    @Override
    public void update() {
        if (timer.hasElapsed(duration) && !isRunning){
            isRunning = true;
            swerve.followPath(mTrajectory);
        }
    }

    @Override
    public void done() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
        timer.stop();
        swerve.stop();
    }

}
