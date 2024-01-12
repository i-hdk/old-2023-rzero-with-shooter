package raidzero.robot.submodules;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.geometry.Rotation2d;
import raidzero.robot.Constants;
import raidzero.robot.Constants.WristConstants;

public class Wrist extends Submodule {
    private Wrist() {
    }

    private static Wrist instance = null;

    public static Wrist getInstance() {
        if (instance == null) {
            instance = new Wrist();
        }
        return instance;
    }

    private enum ControlState {
        OPEN_LOOP, CLOSED_LOOP
    }

    private ControlState mControlState = ControlState.OPEN_LOOP;

    private double mPercentOut = 0.0;
    private double mDesiredAngle = 0.0;
    private double zeroOffset = 0.0;

    private NetworkTable table;
    private DoubleArrayPublisher limitEncoderDataPub;
    private DoubleArraySubscriber limitSwitchEdgeSub;
    private double lastFallingEdge = WristConstants.LIMITSWITCHPOSITIONS[0];
    private Rotation2d wristAngle;

    private final CANSparkMax mMotor = new CANSparkMax(WristConstants.ID, MotorType.kBrushless);
    private final RelativeEncoder mEncoder = mMotor.getEncoder();

    private final SparkMaxLimitSwitch inZoneLimitSwitch = mMotor
            .getForwardLimitSwitch(Constants.WristConstants.LIMITSWITCHPOLARITY);
    private final SparkMaxPIDController mPIDController = mMotor.getPIDController();

    @Override
    public void onInit() {
        mMotor.restoreFactoryDefaults();

        configWristSparkMax();
        // zero();
        limitEncoderDataPub = getDoubleArrayTopic("LimitSwitchData").publish();
        limitSwitchEdgeSub = getDoubleArrayTopic("EdgeData").subscribe(WristConstants.LIMITSWITCHPOSITIONS); // FIX
    }

    @Override
    public void onStart(double timestamp) {
    }

    @Override
    public void update(double timestamp) {
    }

    private void align() {
        double[] defaultEdgeData = { WristConstants.LIMITSWITCHPOSITIONS[0], 1.0 };
        double[] edgeData = limitSwitchEdgeSub.get(defaultEdgeData);
        double fallingEdge = edgeData[1] > 0 ? edgeData[0] : edgeData[0] + WristConstants.LIMITSWITCHDIFFERENCE;
        // System.out.println(fallingEdge);
        if (Math.abs(fallingEdge - lastFallingEdge) > .1)
            mEncoder.setPosition(mEncoder.getPosition() - (fallingEdge - WristConstants.LIMITSWITCHPOSITIONS[0]));

        lastFallingEdge = fallingEdge;

    }

    @Override
    public void run() {
        if (mControlState == ControlState.OPEN_LOOP) {
            mMotor.set(mPercentOut);
        } else if (mControlState == ControlState.CLOSED_LOOP) {
            mPIDController.setReference(
                    mDesiredAngle / WristConstants.POSITION_CONVERSION_FACTOR,
                    ControlType.kSmartMotion,
                    WristConstants.SMART_MOTION_SLOT,
                    0,
                    ArbFFUnits.kPercentOut);
        }
        double limitSwitchEncoderData[] = { inZoneLimitSwitch.isPressed() ? 1 : 0, mMotor.getEncoder().getPosition() };
        // System.out.println(mMotor.getEncoder().getPosition());

        limitEncoderDataPub.set(limitSwitchEncoderData);

        // align();
    }

    @Override
    public void stop() {
        mMotor.stopMotor();
    }

    @Override
    public void zero() {
        mEncoder.setPosition(0);
        wristAngle = Rotation2d.fromDegrees(0);
    }

    /**
     * Sets desired percent speed [-1, 1]
     * 
     * @param speed percent speed
     */
    public void setPercentSpeed(double speed) {
        mControlState = ControlState.OPEN_LOOP;
        mPercentOut = speed;
    }

    /**
     * Set desired wrist angle (degrees) [0, 360]
     * 
     * @param angle desired angle
     */
    public void setDesiredAngle(double angle) {
        mControlState = ControlState.CLOSED_LOOP;
        mDesiredAngle = angle;
        mEncoder.getVelocity();
    }

    /**
     * Get current wrist angle
     * 
     * @return current angle
     */
    public double getRotations() {
        return mEncoder.getPosition();
    }

    /**
     * Get current wrist velocity
     * 
     * @return current velocity
     */
    public double getVelocity() {
        return mEncoder.getVelocity();
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(mEncoder.getPosition() * WristConstants.POSITION_CONVERSION_FACTOR);
    }

    private void configWristSparkMax() {
        mMotor.restoreFactoryDefaults();
        mMotor.setIdleMode(IdleMode.kBrake);
        mMotor.setInverted(WristConstants.INVERSION);
        mMotor.setSmartCurrentLimit(WristConstants.CURRENT_LIMIT);
        mMotor.enableVoltageCompensation(Constants.VOLTAGE_COMP);

        inZoneLimitSwitch.enableLimitSwitch(false);
        // mMotor.enableSoftLimit(SoftLimitDirection.kForward,
        // WristConstants.ENABLEFORWARDLIMIT);
        // mMotor.enableSoftLimit(SoftLimitDirection.kReverse,
        // WristConstants.ENABLEREVERSELIMIT);
        // mMotor.setSoftLimit(SoftLimitDirection.kForward,
        // WristConstants.FORWARDLIMIT);
        // mMotor.setSoftLimit(SoftLimitDirection.kReverse,
        // WristConstants.REVERSELIMIT);

        // mEncoder.setInverted(WristConstants.ENCODER_INVERSION);
        // mEncoder.setPositionConversionFactor(WristConstants.POSITION_CONVERSION_FACTOR);
        // mEncoder.setPositionConversionFactor(1.0);
        // mEncoder.setVelocityConversionFactor(WristConstants.VELOCITY_CONVERSION_FACTOR);

        mPIDController.setFeedbackDevice(mEncoder);
        mPIDController.setFF(WristConstants.KF, WristConstants.SMART_MOTION_SLOT);
        mPIDController.setP(WristConstants.KP, WristConstants.SMART_MOTION_SLOT);
        mPIDController.setI(WristConstants.KI, WristConstants.SMART_MOTION_SLOT);
        mPIDController.setD(WristConstants.KD, WristConstants.SMART_MOTION_SLOT);
        mPIDController.setSmartMotionMinOutputVelocity(WristConstants.MIN_VEL, WristConstants.SMART_MOTION_SLOT);
        mPIDController.setSmartMotionMaxVelocity(WristConstants.MAX_VEL, WristConstants.SMART_MOTION_SLOT);
        mPIDController.setSmartMotionMaxAccel(WristConstants.MAX_ACCEL, WristConstants.SMART_MOTION_SLOT);
        mPIDController.setSmartMotionAllowedClosedLoopError(WristConstants.MIN_ERROR, WristConstants.SMART_MOTION_SLOT);
        mPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, WristConstants.SMART_MOTION_SLOT);
        mPIDController.setPositionPIDWrappingEnabled(true);
        mPIDController.setPositionPIDWrappingMinInput(WristConstants.PID_WRAPPING_MIN);
        mPIDController.setPositionPIDWrappingMaxInput(WristConstants.PID_WRAPPING_MAX);
    }

    private DoubleArrayTopic getDoubleArrayTopic(String key) {
        if (table == null) {
            NetworkTableInstance.getDefault();
            table = NetworkTableInstance.getDefault().getTable(Constants.NETWORKTABLESNAME);
        }
        return table.getDoubleArrayTopic(key);
    }

    public void configSmartMotionConstraints(double wristMaxVel, double wristMaxAccel) {
        mPIDController.setSmartMotionMaxVelocity(wristMaxVel, WristConstants.SMART_MOTION_SLOT);
        mPIDController.setSmartMotionMaxAccel(wristMaxAccel, WristConstants.SMART_MOTION_SLOT);
    }

}
