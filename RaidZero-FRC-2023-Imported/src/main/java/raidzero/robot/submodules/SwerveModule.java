package raidzero.robot.submodules;

import raidzero.robot.Constants;
import raidzero.robot.Constants.SwerveConstants;
import raidzero.robot.utils.MathTools;
import raidzero.robot.wrappers.LazyTalonFX;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SwerveModule extends Submodule implements Sendable {

    private enum ControlState {
        VELOCITY, PATHING, TESTING, PERCENT
    };

    public LazyTalonFX throttle;
    public LazyTalonFX rotor;

    private CANCoder rotorExternalEncoder;

    // The quadrant the module is in (cartesian plane)
    private int quadrant;
    private double forwardAngle = 0.0;

    private double outputThrottleVelocity = 0.0;
    private double outputThrottlePercentSpeed = 0.0;
    private double outputRotorAngle = 0.0;
    private double outputRotorPercentSpeed = 0.0;

    private ControlState controlState = ControlState.VELOCITY;

    @Override
    public void onInit() {
        throw new RuntimeException("Cannot initialize SwerveModule by itself.");
    }

    /**
     * Called once when the submodule is initialized.
     */
    public void onInit(int throttleId, int rotorId, int rotorEncoderId, double forwardAngle) {
        this.forwardAngle = forwardAngle;

        throttle = new LazyTalonFX(throttleId, Constants.CANBUS_STRING);
        initThrottle(throttle);

        // rotorExternalEncoder = new CANCoder(quadrant);
        rotorExternalEncoder = new CANCoder(rotorEncoderId, Constants.CANBUS_STRING);
        rotorExternalEncoder.configFactoryDefault();
        rotorExternalEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360, Constants.TIMEOUT_MS);
        rotorExternalEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToZero,
                Constants.TIMEOUT_MS);
        // rotorExternalEncoder.configMagnetOffset(forwardAngle, Constants.TIMEOUT_MS);

        rotor = new LazyTalonFX(rotorId, Constants.CANBUS_STRING);
        initRotor(rotor, rotorExternalEncoder);

        stop();
    }

    /**
     * Called at the start of autonomous or teleop.
     * 
     * @param timestamp
     */
    @Override
    public void onStart(double timestamp) {
        controlState = ControlState.VELOCITY;
        outputThrottleVelocity = 0.0;
        outputThrottlePercentSpeed = 0.0;
        outputRotorAngle = 0.0;
    }

    /**
     * Reads cached inputs & calculate outputs.
     */
    @Override
    public void update(double timestamp) {
    }

    @Override
    public void stop() {
        outputThrottleVelocity = 0.0;
        outputThrottlePercentSpeed = 0.0;
        outputRotorAngle = getRotorAngle();
        outputRotorPercentSpeed = 0.0;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Gyro");
        builder.addDoubleProperty("Value", this::getNegatedRotorAngle, null);
    }

    @Override
    public void zero() {
        throttle.setSelectedSensorPosition(0, SwerveConstants.THROTTLE_VELOCITY_PID_SLOT, Constants.TIMEOUT_MS);
        double abs = rotorExternalEncoder.getAbsolutePosition();
        System.out.println("Q" + quadrant + " abs angle=" + abs + ", forward=" + forwardAngle);
        rotorExternalEncoder.setPosition(MathTools.wrapDegrees(abs - forwardAngle));
    }

    /** Runs components in the submodule that have continuously changing inputs. */
    public void run() {
        switch (controlState) {
            case VELOCITY:
                throttle.set(ControlMode.Velocity, outputThrottleVelocity);
                rotor.set(ControlMode.Position, outputRotorAngle);
                break;
            case PATHING:
                break;
            case TESTING:
                throttle.set(ControlMode.PercentOutput, outputThrottlePercentSpeed);
                rotor.set(ControlMode.PercentOutput, outputRotorPercentSpeed);
                break;
            case PERCENT:
                throttle.set(ControlMode.PercentOutput, outputThrottlePercentSpeed);
                rotor.set(ControlMode.Position, outputRotorAngle);
                break;
        }
    }

    /**
     * Returns the velocity of the throttle in meters per second.
     * 
     * @return the velocity of the throttle in meters per second.
     */
    public double getThrottleVelocity() {
        return throttle.getSelectedSensorVelocity(SwerveConstants.THROTTLE_VELOCITY_PID_SLOT)
                * SwerveConstants.THROTTLE_TICKS_TO_METERS * 10.0;
    }

    /**
     * Sets the velocity of the throttle.
     * 
     * @param velocity target throttle velocity in meters per second.
     */
    public void setThrottleVelocity(double velocity) {
        controlState = ControlState.VELOCITY;

        outputThrottleVelocity = velocity / (SwerveConstants.THROTTLE_TICKS_TO_METERS * 10.0);
    }

    /**
     * Returns percent speed of throttle [-1, 1]
     * 
     * @return throttle percent speed
     */
    public double getThrottlePercentSpeed() {
        return throttle.getMotorOutputPercent();
    }

    /**
     * Sets percent speed of throttle
     * 
     * @param percentSpeed percent speed
     */
    public void setThrottlePercentSpeed(double percentSpeed) {
        controlState = ControlState.PERCENT;
        outputThrottlePercentSpeed = percentSpeed;
    }

    /**
     * Returns relative throttle position
     * 
     * @return thottle position
     */
    public double getThrottlePosition() {
        return throttle.getSelectedSensorPosition() * SwerveConstants.THROTTLE_TICKS_TO_METERS;
    }

    /**
     * Returns the angle of the rotor (+ is ccw) in degrees.
     * 
     * @return angle of rotor in degrees.
     */
    public double getRotorAngle() {
        return rotorExternalEncoder.getPosition();
    }

    /**
     * Returns the negated angle of the rotor (+ is cw) in degrees.
     * 
     * @return angle of rotor in degrees.
     */
    public double getNegatedRotorAngle() {
        return -getRotorAngle();
    }

    /**
     * Sets the angle of the rotor.
     * 
     * @param angle target rotor angle in degrees.
     */
    public void setRotorAngle(double angle) {
        double currentAngle = getRotorAngle();
        double delta = angle - currentAngle;
        delta = delta % 360;
        while (delta > 180)
            delta -= 360;
        while (delta < -180)
            delta += 360;

        outputRotorAngle = (currentAngle + delta) / SwerveConstants.CANCODER_TO_DEGREES;
    }

    /**
     * Returns the current state of the swerve module.
     * 
     * @return state of swerve module
     */
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getThrottleVelocity(), Rotation2d.fromDegrees(getRotorAngle()));
    }

    /**
     * Returns swerve module position
     * 
     * @return swerve module position
     */
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getThrottlePosition(), Rotation2d.fromDegrees(getRotorAngle()));
    }

    /**
     * Sets the target state of the module.
     * 
     * @param targetState target state of the module
     */
    public void setTargetState(SwerveModuleState targetState) {
        setTargetState(targetState, false, true, false);
    }

    /**
     * Sets the target state of the swerve module to the provided one.
     * 
     * @param targetState the target state
     * @param ignoreAngle whether to ignore the target angle
     * @param optimize    whether to optimize the target angle
     */
    public void setTargetState(SwerveModuleState targetState, boolean ignoreAngle, boolean optimize,
            boolean isOpenLoop) {
        SwerveModuleState state = targetState;
        if (optimize) {
            // Optimize the reference state to avoid spinning further than 90 degrees
            state = SwerveModuleState.optimize(targetState,
                    Rotation2d.fromDegrees(MathTools.wrapDegrees(getRotorAngle())));
        }
        // System.out.println("Q" + quadrant + ": state=" + state);
        if (isOpenLoop) {
            setThrottlePercentSpeed(state.speedMetersPerSecond);
        } else {
            setThrottleVelocity(state.speedMetersPerSecond);
        }
        if (!ignoreAngle) {
            setRotorAngle(state.angle.getDegrees());
        }
    }

    /**
     * Returns the smallest rotor angle error.
     * 
     * @return error in degrees
     */
    public double getRotorAngleError() {
        double error = (rotorExternalEncoder.getPosition() - outputRotorAngle * SwerveConstants.CANCODER_TO_DEGREES)
                % 360.0;
        if (Math.abs(error) > 180) {
            if (error > 0) {
                return error - 180;
            } else {
                return error + 180;
            }
        }
        return error;
    }

    public void testThrottleAndRotor(double throttleOutput, double rotorOutput) {
        controlState = ControlState.TESTING;

        outputThrottlePercentSpeed = throttleOutput;
        outputRotorPercentSpeed = rotorOutput;
    }

    public void initThrottle(TalonFX throttle) {
        throttle.configFactoryDefault();
        throttle.setInverted(SwerveConstants.MOTOR_INVERSION);
        throttle.setNeutralMode(NeutralMode.Brake);
        throttle.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        throttle.config_kF(SwerveConstants.THROTTLE_VELOCITY_PID_SLOT, SwerveConstants.THROTTLE_KF);
        throttle.config_kP(SwerveConstants.THROTTLE_VELOCITY_PID_SLOT, SwerveConstants.THROTTLE_KP);
        throttle.config_kD(SwerveConstants.THROTTLE_VELOCITY_PID_SLOT, SwerveConstants.THROTTLE_KD);
        throttle.configVoltageCompSaturation(Constants.VOLTAGE_COMP);
        throttle.enableVoltageCompensation(true);
        throttle.configSupplyCurrentLimit(SwerveConstants.THROTTLE_CURRENT_LIMIT);

        throttle.configOpenloopRamp(SwerveConstants.kOpenLoopRampRate);
        throttle.configClosedloopRamp(SwerveConstants.kClosedLoopRampRate);
    }

    public void initRotor(TalonFX rotor, CANCoder encoder) {
        rotor.configFactoryDefault();
        rotor.setInverted(SwerveConstants.ROTOR_INVERSION);
        rotor.setNeutralMode(NeutralMode.Brake);
        rotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
        rotor.setSensorPhase(SwerveConstants.ROTOR_INVERT_SENSOR_PHASE);
        rotor.configRemoteFeedbackFilter(encoder, 0);
        rotor.config_kP(SwerveConstants.ROTOR_POSITION_PID_SLOT, SwerveConstants.ROTOR_KP);
        rotor.config_kI(SwerveConstants.ROTOR_POSITION_PID_SLOT, SwerveConstants.ROTOR_KI);
        rotor.config_kD(SwerveConstants.ROTOR_POSITION_PID_SLOT, SwerveConstants.ROTOR_KD);
        rotor.configVoltageCompSaturation(Constants.VOLTAGE_COMP);
        rotor.enableVoltageCompensation(true);
        rotor.configSupplyCurrentLimit(SwerveConstants.ROTOR_CURRENT_LIMIT);
    }
}
