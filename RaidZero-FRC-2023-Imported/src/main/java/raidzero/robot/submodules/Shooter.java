package raidzero.robot.submodules;

import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import raidzero.robot.Constants;
import raidzero.robot.Constants.IntakeConstants;
import raidzero.robot.wrappers.LazyTalonFX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
//rev 2024 not out yet

public class Shooter extends Submodule {

    private static Shooter instance = null;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    private Shooter() {
    }

    private final double FAKE_MAX_SPEED = 17000;
    private final double ERROR_TOLERANCE = 250;
    private final int MOTOR_LEFT_ID;
    private final int MOTOR_RIGHT_ID;
    private final int PID_SLOT;

    private final CANSparkMax mLeftMotor = new CANSparkMax(MOTOR_LEFT_ID, MotorType.kBrushless);
    private final RelativeEncoder mLeftEncoder = mLeftMotor.getEncoder();
    private final SparkMaxPIDController mLeftPIDController = mLeftMotor.getPIDController();
    private final CANSparkMax mRightMotor = new CANSparkMax(MOTOR_RIGHT_ID, MotorType.kBrushless);
    private final RelativeEncoder mRightEncoder = mRightMotor.getEncoder();
    private final SparkMaxPIDController mRightPIDController = mRightMotor.getPIDController();

    private double outputPercentSpeed = 0.0;

    @Override
    public void onInit() {
        mLeftMotor.restoreFactoryDefaults();
        mRightMotor.restoreFactoryDefaults();
        configShooterSparkMax();

    }

    @Override
    public void onStart(double timestamp) {
        outputPercentSpeed = 0.0;
        zero();
    }

    @Override
    public void update(double timestamp) {
    }

    @Override
    public void run() {
        if (Math.abs(outputPercentSpeed) < 0.1) {
            stop();
        } else {
            mLeftPIDController.setReference(
                outputPercentSpeed*FAKE_MAX_SPEED,
                ControlType.kSmartVelocity,
                PID_SLOT 
            );
            mRightPIDController.setReference(
                outputPercentSpeed*FAKE_MAX_SPEED,
                ControlType.kSmartVelocity,
                PID_SLOT 
            );
        }
    }

    @Override
    public void stop() {
        mLeftMotor.stopMotor();
        mRightMotor.stopMotor();
    }

    @Override
    public void zero() {
        mLeftEncoder.setPosition(0);
        mRightEncoder.setPosition(0);
    }

    /**
     * Fires up the shooter.
     * 
     * @param percentSpeed speed of the shooter in [-1.0, 1.0]
     * @param freeze       whether to disregard the speed and keep the previous speed
     */
    public void shoot(double percentSpeed, boolean freeze) {
        if (freeze) {
            return;
        }
        outputPercentSpeed = percentSpeed;
    }

    public void configSmartCurrentLimit(int stall, int free, int stallrpm) {
        mLeftMotor.setSmartCurrentLimit(stall, free, stallrpm);
        mRightMotor.setSmartCurrentLimit(stall, free, stallrpm);
    }

    private void configShooterSparkMax(){
        mLeftMotor.restoreFactoryDefaults();
        mLeftMotor.setIdleMode(IdleMode.kBrake);
        mLeftMotor.setInverted(true);
        mLeftMotor.setSmartCurrentLimit(IntakeConstants.STALL_CURRENT_LIMIT, IntakeConstants.FREE_CURRENT_LIMIT, IntakeConstants.STALL_RPM); //change this??
        mLeftMotor.enableVoltageCompensation(Constants.VOLTAGE_COMP);
        mLeftPIDController.setFeedbackDevice(mEncoder);
        mLeftPIDController.setFF(IntakeConstants.KF, IntakeConstants.PID_SLOT);
        mLeftPIDController.setP(IntakeConstants.KP, IntakeConstants.PID_SLOT);
        mLeftPIDController.setI(IntakeConstants.KI, IntakeConstants.PID_SLOT);
        mLeftPIDController.setD(IntakeConstants.KD, IntakeConstants.PID_SLOT);
        mLeftPIDController.setSmartMotionMinOutputVelocity(IntakeConstants.MIN_VEL, IntakeConstants.SMART_MOTION_SLOT);
        mLeftPIDController.setSmartMotionMaxVelocity(IntakeConstants.MAX_VEL, IntakeConstants.SMART_MOTION_SLOT);
        mLeftPIDController.setSmartMotionMaxAccel(IntakeConstants.MAX_ACCEL, IntakeConstants.SMART_MOTION_SLOT);
        mLeftPIDController.setSmartMotionAllowedClosedLoopError(IntakeConstants.MIN_ERROR, IntakeConstants.SMART_MOTION_SLOT);
        mLeftPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, IntakeConstants.SMART_MOTION_SLOT);
        mRightMotor.restoreFactoryDefaults();
        mRightMotor.setIdleMode(IdleMode.kBrake);
        mRightMotor.setInverted(false);
        mRightMotor.setSmartCurrentLimit(IntakeConstants.STALL_CURRENT_LIMIT, IntakeConstants.FREE_CURRENT_LIMIT, IntakeConstants.STALL_RPM); //change this??
        mRightMotor.enableVoltageCompensation(Constants.VOLTAGE_COMP);
        mRightPIDController.setFeedbackDevice(mEncoder);
        mRightPIDController.setFF(IntakeConstants.KF, IntakeConstants.PID_SLOT);
        mRightPIDController.setP(IntakeConstants.KP, IntakeConstants.PID_SLOT);
        mRightPIDController.setI(IntakeConstants.KI, IntakeConstants.PID_SLOT);
        mRightPIDController.setD(IntakeConstants.KD, IntakeConstants.PID_SLOT);
        mRightPIDController.setSmartMotionMinOutputVelocity(IntakeConstants.MIN_VEL, IntakeConstants.SMART_MOTION_SLOT);
        mRightPIDController.setSmartMotionMaxVelocity(IntakeConstants.MAX_VEL, IntakeConstants.SMART_MOTION_SLOT);
        mRightPIDController.setSmartMotionMaxAccel(IntakeConstants.MAX_ACCEL, IntakeConstants.SMART_MOTION_SLOT);
        mRightPIDController.setSmartMotionAllowedClosedLoopError(IntakeConstants.MIN_ERROR, IntakeConstants.SMART_MOTION_SLOT);
        mRightPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, IntakeConstants.SMART_MOTION_SLOT);
    }
}