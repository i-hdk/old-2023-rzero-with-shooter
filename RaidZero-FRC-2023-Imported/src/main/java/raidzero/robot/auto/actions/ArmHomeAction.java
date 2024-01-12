package raidzero.robot.auto.actions;

import edu.wpi.first.math.geometry.Pose2d;
import raidzero.robot.submodules.Arm;

public class ArmHomeAction implements Action {
    private static final Arm mArm = Arm.getInstance();

    public ArmHomeAction() {
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
        Pose2d endPose = mArm.getState()[1];
        return Math.abs(endPose.getX()) < 0.15 && Math.abs(endPose.getY() - 0.15) < 0.15;
    }
}
