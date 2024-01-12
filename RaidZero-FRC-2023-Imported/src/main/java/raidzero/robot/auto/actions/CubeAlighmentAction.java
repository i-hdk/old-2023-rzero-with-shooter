package raidzero.robot.auto.actions;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import raidzero.robot.submodules.Swerve;
import raidzero.robot.submodules.Vision;


public class CubeAlighmentAction implements Action {

    private static final Swerve swerve = Swerve.getInstance();
    private Transform2d transform;
    private Pose2d updatePose;
    private Pose2d endPose;
    private Rotation2d cubeAngle;
    private Timer timer = new Timer();
    private Vision vision = Vision.getInstance();

    public CubeAlighmentAction(Pose2d endPose) {
        this.endPose = endPose;
    }

    @Override
    public boolean isFinished() {
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
        Translation2d relativeTranslation = swerve.getPose().minus(endPose).getTranslation();
        transform = swerve.getPose().minus(endPose);
        cubeAngle = new Rotation2d(Math.toRadians(vision.getCubeAngle()));
        transform = transform.plus(new Transform2d(new Translation2d(), transform.getRotation().minus(cubeAngle)));
        updatePose = swerve.getPose().plus(transform);
        Pose2d relativeRobotPose = new Pose2d(relativeTranslation, cubeAngle);

        swerve.addVisionMeasurement(relativeRobotPose.plus(transform), Timer.getFPGATimestamp(),
                new MatBuilder<N3, N1>(Nat.N3(), Nat.N1()).fill(
                        0.01 * relativeTranslation.getNorm(),
                        0.01 * relativeTranslation.getNorm(), 1.0));
    }

    @Override
    public void done() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
        timer.stop();
    }
}
