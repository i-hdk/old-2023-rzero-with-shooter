package raidzero.robot.auto.actions;

import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;

import edu.wpi.first.wpilibj.Timer;
import raidzero.robot.submodules.Vision;

public class WaitForFlyingCube implements Action {
    private Timer mTimer = new Timer();
    private static final Vision vision = Vision.getInstance();

    public WaitForFlyingCube() {
    }

    @Override
    public void start() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");
        mTimer.reset();
        mTimer.start();
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
        mTimer.stop();
    }

    @Override
    public boolean isFinished() {
        return vision.getCubeAngle()!=0 && vision.getCubeX()!=0 && vision.getCubeY()!=0;
    }
}
