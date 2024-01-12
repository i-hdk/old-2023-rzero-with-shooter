package raidzero.robot.submodules;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
//rev 2024 not out yet

import raidzero.robot.Constants;
import raidzero.robot.Constants.IntakeConstants;

public class Intake extends Submodule {
    private Intake() {
    }

    private static Intake instance = null;

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    private enum ControlState {
        OPEN_LOOP, CLOSED_LOOP
    }

    private ControlState mControlState = ControlState.OPEN_LOOP;

    private double mPercentOut = 0.0;
    private double mDesiredPosition = 0.0;
    private double mPrevOpenLoopPosition = 0.0;

    private final CANSparkMax mMotor = new CANSparkMax(IntakeConstants.ID, MotorType.kBrushless);
    private final RelativeEncoder mEncoder = mMotor.getEncoder();
    private final SparkMaxPIDController mPIDController = mMotor.getPIDController();

    @Override
    public void onInit() {
        mMotor.restoreFactoryDefaults();
        configIntakeSparkMax();
        zero();
    }

    @Override
    public void onStart(double timestamp) {
    }

    @Override
    public void update(double timestamp) {
        // SmartDashboard.putNumber("Intake current draw", mMotor.getOutputCurrent());
    }

    @Override
    public void run() {
        if (Math.abs(mPercentOut) < 0.05) {
            holdPosition();
        }
        if (mControlState == ControlState.OPEN_LOOP) {
            mMotor.set(mPercentOut);
            mPrevOpenLoopPosition = mEncoder.getPosition();
        } else if (mControlState == ControlState.CLOSED_LOOP) {
            mPIDController.setReference(
                    mDesiredPosition,
                    ControlType.kPosition,
                    IntakeConstants.PID_SLOT);
        }
    }

    @Override
    public void stop() {
        mMotor.stopMotor();
    }

    @Override
    public void zero() {
        mEncoder.setPosition(0);
    }

    /**
     * Set intake percent speed [-1, 1]
     * 
     * @param speed percent speed
     */
    public void setPercentSpeed(double speed) {
        mControlState = ControlState.OPEN_LOOP;
        mPercentOut = speed;
    }

    /** Hold position of intake */
    public void holdPosition() {
        mControlState = ControlState.CLOSED_LOOP;
        if (Math.signum(mPercentOut) < 0)
            mDesiredPosition = mPrevOpenLoopPosition - 1;
        else
            mDesiredPosition = mPrevOpenLoopPosition + 1;
    }

    public int getEncoderPosition(){
        return mEncoder.getPosition();
    }

    public void configSmartCurrentLimit(int stall, int free, int stallrpm) {
        mMotor.setSmartCurrentLimit(stall, free, stallrpm);
    }

    /** Configure intake motor & integrated encoder/PID controller */
    private void configIntakeSparkMax() {
        mMotor.restoreFactoryDefaults();
        mMotor.setIdleMode(IdleMode.kBrake);
        mMotor.setInverted(IntakeConstants.INVERSION);
        mMotor.setSmartCurrentLimit(IntakeConstants.STALL_CURRENT_LIMIT, IntakeConstants.FREE_CURRENT_LIMIT,
                IntakeConstants.STALL_RPM);
        mMotor.enableVoltageCompensation(Constants.VOLTAGE_COMP);

        mPIDController.setFeedbackDevice(mEncoder);
        mPIDController.setFF(IntakeConstants.KF, IntakeConstants.PID_SLOT);
        mPIDController.setP(IntakeConstants.KP, IntakeConstants.PID_SLOT);
        mPIDController.setI(IntakeConstants.KI, IntakeConstants.PID_SLOT);
        mPIDController.setD(IntakeConstants.KD, IntakeConstants.PID_SLOT);

        mPIDController.setSmartMotionMinOutputVelocity(IntakeConstants.MIN_VEL, IntakeConstants.SMART_MOTION_SLOT);
        mPIDController.setSmartMotionMaxVelocity(IntakeConstants.MAX_VEL, IntakeConstants.SMART_MOTION_SLOT);
        mPIDController.setSmartMotionMaxAccel(IntakeConstants.MAX_ACCEL, IntakeConstants.SMART_MOTION_SLOT);
        mPIDController.setSmartMotionAllowedClosedLoopError(IntakeConstants.MIN_ERROR,
                IntakeConstants.SMART_MOTION_SLOT);
        mPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, IntakeConstants.SMART_MOTION_SLOT);
    }
}