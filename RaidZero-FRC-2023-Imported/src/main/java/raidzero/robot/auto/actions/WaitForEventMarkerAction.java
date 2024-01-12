package raidzero.robot.auto.actions;

import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;

import edu.wpi.first.wpilibj.Timer;

public class WaitForEventMarkerAction implements Action {
    private Timer mTimer = new Timer();
    private List<EventMarker> mMarkers;
    private double mWaitTime;

    /**
     * Action that blocks until event marker has been reached
     * 
     * @param trajectory  the path the robot is following
     * @param name        marker name
     * @param currentTime current time in the trajectory
     */
    public WaitForEventMarkerAction(PathPlannerTrajectory trajectory, String name, double currentTime) {
        mMarkers = trajectory.getMarkers();
        double wait = 0.0;
        for (EventMarker marker : mMarkers) {
            if (marker.names.get(0).equals(name)) {
                wait = marker.timeSeconds - currentTime;
            }
        }
        mWaitTime = wait > 0 ? wait : 0;
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
        return mTimer.hasElapsed(mWaitTime);
    }
}
