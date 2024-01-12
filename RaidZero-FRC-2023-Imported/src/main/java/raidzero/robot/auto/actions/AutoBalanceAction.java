package raidzero.robot.auto.actions;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import raidzero.robot.submodules.Intake;
import raidzero.robot.submodules.Swerve;

public class AutoBalanceAction implements Action {
    private static final Swerve mSwerve = Swerve.getInstance();
    private Timer timer = new Timer();
    private Alliance alliance;
    private boolean blue = false;
    private double mDuration, mSpeed, reverse;
    private double threshold;

    public AutoBalanceAction(boolean opp, double t) {
        blue = DriverStation.getAlliance() == Alliance.Blue;
        reverse = opp ? (blue ? 1 : -1) : (blue ? -1 : 1);
        threshold = t;
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(mSwerve.getBeans()) > threshold);
    }

    @Override
    public void start() {
        mSwerve.emptyBucket();
        mSwerve.drive(reverse*0.2, 0, 0, true);
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
        mSwerve.stop();
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
    }
}
