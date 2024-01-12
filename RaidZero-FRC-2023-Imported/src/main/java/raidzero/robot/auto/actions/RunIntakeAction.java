package raidzero.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;

import raidzero.robot.submodules.Intake;

public class RunIntakeAction implements Action {
    private static final Intake mIntake = Intake.getInstance();
    private Timer timer = new Timer();

    private double mDuration, mSpeed;

    public RunIntakeAction(double duration, double speed) {
        mDuration = duration;
        mSpeed = speed;
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(mDuration);
    }

    @Override
    public void start() {
        timer.reset();
        timer.start();
        mIntake.setPercentSpeed(mSpeed);
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
        timer.stop();
        mIntake.setPercentSpeed(0);
    }
}
