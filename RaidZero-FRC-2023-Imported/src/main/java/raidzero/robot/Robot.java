package raidzero.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import raidzero.robot.auto.AutoRunner;
import raidzero.robot.submodules.*;
import raidzero.robot.submodules.SubmoduleManager;
import raidzero.robot.teleop.Teleop;

/**
 * The main robot class.
 */
public class Robot extends TimedRobot {
    private static final SubmoduleManager submoduleManager = SubmoduleManager.getInstance();

    private static final Teleop teleop = Teleop.getInstance();
    private static final Swerve swerve = Swerve.getInstance();
    private static final Vision vision = Vision.getInstance();
    private static final Intake intake = Intake.getInstance();
    private static final Arm arm = Arm.getInstance();
    private static final Wrist wrist = Wrist.getInstance();
    private static final Lights lights = Lights.getInstance();

    private AutoRunner autoRunner;

    // private NetworkTableEntry shouldCheckGSCEntry =
    // Shuffleboard.getTab(Tab.SELECTION).add("Scan Path",
    // 0).withWidget(BuiltInWidgets.kBooleanBox)
    // .withSize(1, 1).withPosition(0, 0).getEntry();
    // private NetworkTableEntry foundPathEntry =
    // Shuffleboard.getTab(Tab.SELECTION).add("Chosen Path",
    // 0).withWidget(BuiltInWidgets.kTextView)
    // .withSize(2, 1).withPosition(0, 1).getEntry();

    /**
     * Runs only once at the start of robot code execution.
     */
    @Override
    public void robotInit() {
        // Register all submodules here
        submoduleManager.setSubmodules(
                swerve,
                arm,
                wrist,
                intake,
                vision, 
                lights
        );
        submoduleManager.onInit();

        autoRunner = new AutoRunner();
    }

    /**
     * Runs every time the robot is disabled.
     */
    @Override
    public void disabledInit() {
        // Limelight.getInstance().setLedMode(LedMode.Off);

        // Stop autonomous
        autoRunner.stop();
        submoduleManager.onStop(Timer.getFPGATimestamp());
    }

    /**
     * Runs at the start of autonomous.
     */
    @Override
    public void autonomousInit() {

        autoRunner.readSendableSequence();
        autoRunner.start();
        submoduleManager.onStart(Timer.getFPGATimestamp());
    }

    /**
     * Runs every 0.02s during autonomous (50 Hz).
     */
    @Override
    public void autonomousPeriodic() {
        double timestamp = Timer.getFPGATimestamp();
        // System.out.println("tx full: " + RobotController.getCANStatus().txFullCount);
        submoduleManager.onLoop(timestamp);
        autoRunner.onLoop(timestamp);

    }

    /**
     * Runs at the start of teleop.
     */
    @Override
    public void teleopInit() {
        // Stop the autonomous
        autoRunner.stop();

        // Start the teleop handler
        teleop.onStart();
        submoduleManager.onStart(Timer.getFPGATimestamp());
    }

    /**
     * Runs every 0.02s during teleop (50 Hz).
     */
    @Override
    public void teleopPeriodic() {
        teleop.onLoop();
        submoduleManager.onLoop(Timer.getFPGATimestamp());
    }
}