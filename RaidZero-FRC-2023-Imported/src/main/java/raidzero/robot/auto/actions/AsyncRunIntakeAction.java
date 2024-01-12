package raidzero.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;

import raidzero.robot.submodules.Intake;

public class AsyncRunIntakeAction implements Action {
    private static final Intake mIntake = Intake.getInstance();
    private double mSpeed;

    public AsyncRunIntakeAction(double speed) {
        mSpeed = speed;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void start() {
        mIntake.setPercentSpeed(mSpeed);
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
    }
}
