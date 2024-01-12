package raidzero.robot.utils;

import raidzero.robot.Constants.VisionConstants;
import raidzero.robot.submodules.Swerve;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Multithreading implements Runnable{

    private double timestamp;
    private NetworkTable cameraSubTable;
    private Pose2d newRobotPose;
    private Matrix<N3, N1> errors;

    private static final Swerve swerve = Swerve.getInstance();

    public Multithreading(Pose2d newRobotPose, double timestamp, Matrix<N3, N1> errors) {
        this.newRobotPose = newRobotPose;
        this.timestamp = timestamp;
        this.errors = errors;
    }

    @Override
    public void run(){
        swerve.addVisionMeasurement(newRobotPose, timestamp, errors);
    }
}
