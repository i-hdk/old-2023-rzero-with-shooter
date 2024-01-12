package raidzero.robot.auto.actions;

import edu.wpi.first.math.geometry.Pose2d;
import raidzero.robot.submodules.Arm;

public class AsyncArmHomeAction implements Action {
    private static final Arm mArm = Arm.getInstance();

    public AsyncArmHomeAction() {
    }

    @Override
    public void start() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");
        mArm.goHome();
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
