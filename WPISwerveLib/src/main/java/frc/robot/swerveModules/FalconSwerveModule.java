package frc.robot.swerveModules;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;

public class FalconSwerveModule {

    private static final double MOTOR_ENCODER_CLICKS_PER_ROTATION = 2048;

    // Gear ratio for mk4i in L2 configuration - SDS library
    private static final double driveGearRatio = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    private static final double steerGearRatio = (14.0 / 50.0) * (10.0 / 60.0);

    private static final double kWheelDiameter = 0.10033;
    private static final double kWheelCircumference = kWheelDiameter * Math.PI;
    private static final double encoderClicksPerRotation = MOTOR_ENCODER_CLICKS_PER_ROTATION * (1 / driveGearRatio);

    private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
    private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

    /**
     * This represents the native max velocity for the drive sensor over 100 ms
     * It should follow the following formula
     * maxRpm * encoderCpr / 600
     * 
     */
    private static final double m_sensorUnitsMaxVelocity = 6000.0 * 2048.0 / 600.0;

    // Velocity PID constants
    private static final int m_gainsSlot = 0;

    /*
     * Feedforward calculated by taking max theoratical gain and then manually
     * tuning
     */
    private static final double m_driveVelF = (1023.0 / 20660.0) + 0.00;

    // TODO: verify pid values
    // PID calculated from SysID tool
    private static final double m_driveVelP = 0.00071716;
    private static final double m_driveVelI = 0.0;
    private static final double m_driveVelD = 0.0;

    private static final double m_nominalOutput = 0;
    private static final double m_peakOutput = 0.5;
    private static final double nominalVoltage = 12;

    private static final int m_timeoutMs = 0;

    // Variables stolen from SDS library for steering (No idea what they do, yet)
    private static final int ENCODER_RESET_ITERATIONS = 500;
    private static final double ENCODER_RESET_MAX_ANGULAR_VELOCITY = Math.toRadians(0.5);
    private double resetIteration = 0;
    private static final double motorEncoderPositionCoefficient = 2.0 * Math.PI / MOTOR_ENCODER_CLICKS_PER_ROTATION
            * steerGearRatio;
    private static final double motorEncoderVelocityCoefficient = motorEncoderPositionCoefficient * 10.0;

    // From sds library
    private static final double m_turnVelP = 0.2;
    private static final double m_turnVelI = 0.0;
    private static final double m_turnVelD = 0.1;

    private static final double m_turnVelF = (1023.0 * motorEncoderVelocityCoefficient / nominalVoltage);

    private final WPI_TalonFX m_driveMotor;
    private final WPI_TalonFX m_turnMotor;

    private final CANCoder m_canCoder;

    // Gains are for example purposes only - must be determined for your own robot!
    private final PIDController m_drivePIDController = new PIDController(0.56122, 0, 0);

    // Gains are for example purposes only - must be determined for your own robot!
    private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
            0.2,
            0,
            0.1,
            new TrapezoidProfile.Constraints(
                    kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

    // Gains are for example purposes only - must be determined for your own robot!
    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
    private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

    public FalconSwerveModule(
            int driveMotorID,
            int turnMotorID,
            int canCoderID, double offSetRadians) {
        m_driveMotor = new WPI_TalonFX(driveMotorID);
        m_turnMotor = new WPI_TalonFX(turnMotorID);
        m_canCoder = new CANCoder(canCoderID);

        m_driveMotor.configFactoryDefault();
        m_turnMotor.configFactoryDefault();
        m_canCoder.configFactoryDefault();

        m_driveMotor.setInverted(true);
        m_turnMotor.setInverted(false);

        m_driveMotor.configNominalOutputForward(m_nominalOutput, m_timeoutMs);
        m_driveMotor.configNominalOutputReverse(-m_nominalOutput, m_timeoutMs);
        m_driveMotor.configPeakOutputForward(m_peakOutput, m_timeoutMs);
        m_driveMotor.configPeakOutputReverse(-m_peakOutput, m_timeoutMs);

        m_driveMotor.config_kF(m_gainsSlot, m_driveVelF, m_timeoutMs);
        m_driveMotor.config_kP(m_gainsSlot, m_driveVelP, m_timeoutMs);
        m_driveMotor.config_kI(m_gainsSlot, m_driveVelI, m_timeoutMs);
        m_driveMotor.config_kD(m_gainsSlot, m_driveVelD, m_timeoutMs);

        m_turnMotor.configNominalOutputForward(m_nominalOutput, m_timeoutMs);
        m_turnMotor.configNominalOutputReverse(-m_nominalOutput, m_timeoutMs);
        m_turnMotor.configPeakOutputForward(m_peakOutput, m_timeoutMs);
        m_turnMotor.configPeakOutputReverse(-m_peakOutput, m_timeoutMs);

        //m_turnMotor.config_kF(m_gainsSlot, m_turnVelF, m_timeoutMs);
        m_turnMotor.config_kP(m_gainsSlot, m_turnVelP, m_timeoutMs);
        m_turnMotor.config_kI(m_gainsSlot, m_turnVelI, m_timeoutMs);
        m_turnMotor.config_kD(m_gainsSlot, m_turnVelD, m_timeoutMs);

        m_canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

        // replace with consturctor/config value
        m_canCoder.configMagnetOffset(Math.toDegrees(offSetRadians), m_timeoutMs);
        m_canCoder.configSensorDirection(false, m_timeoutMs);

        m_turnMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, m_timeoutMs);
        m_turnMotor.setSensorPhase(true);
        m_turnMotor.setSelectedSensorPosition(getAbsoluteAngle() / motorEncoderPositionCoefficient, 0, m_timeoutMs);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                getVelocity(), Rotation2d.fromRadians(getAbsoluteAngle()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                clicksToMeters(m_driveMotor.getSelectedSensorPosition()),
                Rotation2d.fromRadians(getAbsoluteAngle()));
    }

    public void setDesiredStateOld(SwerveModuleState desiredState) {
        SwerveModuleState state = SwerveModuleState.optimize(desiredState,
                Rotation2d.fromRadians(getAbsoluteAngle()));

        final double driveOutput = m_drivePIDController.calculate(getVelocity(), state.speedMetersPerSecond);
        final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

        final double turnOutput = m_turningPIDController.calculate(getAbsoluteAngle(),
                state.angle.getRadians());
        final double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

        m_driveMotor.setVoltage(driveOutput + driveFeedforward);
        m_turnMotor.setVoltage(turnOutput + turnFeedforward);
    }

    public void setDesiredState(SwerveModuleState desiredState) {

        // WPI optimize (doesn't work because ctre control loops are not continous)
        // desiredState = SwerveModuleState.optimize(desiredState,
        // Rotation2d.fromRadians(getAbsoluteAngle()));

        // SDS optimize code (doesn't work)
        double steerAngleRadians = desiredState.angle.getRadians();
        double speedMetersPerSecond = desiredState.speedMetersPerSecond;

        // Optimize
        steerAngleRadians %= (2.0 * Math.PI);
        if (steerAngleRadians < 0.0) {
            steerAngleRadians += 2.0 * Math.PI;
        }

        double difference = steerAngleRadians - getStateAngle();
        // Change the target angle so the difference is in the range [-pi, pi) instead
        // of [0, 2pi)
        if (difference >= Math.PI) {
            steerAngleRadians -= 2.0 * Math.PI;
        } else if (difference < -Math.PI) {
            steerAngleRadians += 2.0 * Math.PI;
        }
        difference = steerAngleRadians - getStateAngle(); // Recalculate difference

        // If the difference is greater than 90 deg or less than -90 deg the drive can
        // be inverted so the total
        // movement of the module is less than 90 deg
        if (difference > Math.PI / 2.0 || difference < -Math.PI / 2.0) {
            // Only need to add 180 deg here because the target angle will be put back into
            // the range [0, 2pi)
            steerAngleRadians += Math.PI;
            speedMetersPerSecond *= -1.0;
        }

        // Put the target angle back into the range [0, 2pi)
        steerAngleRadians %= (2.0 * Math.PI);
        if (steerAngleRadians < 0.0) {
            steerAngleRadians += 2.0 * Math.PI;
        }

        // Team 364's optimize
        // SwerveModuleState state = optimize(desiredState,
        // Rotation2d.fromDegrees(m_canCoder.getAbsolutePosition()));
        // double steerAngleRadians = state.angle.getRadians();
        // double speedMetersPerSecond = state.speedMetersPerSecond;

        // calculate speed
        double driveSensorUnitsPer100Ms = calculateDriveSensorUnitsPer100Ms(speedMetersPerSecond);
        double turnSensorUnitsPosition = calculateTurnSensorUnitsPosition(steerAngleRadians);

        m_driveMotor.set(TalonFXControlMode.Velocity, driveSensorUnitsPer100Ms);
        m_turnMotor.set(TalonFXControlMode.Position, turnSensorUnitsPosition);
    }

    // Heavily based from SDS library setReferenceAngle in
    // Falcon500SteerControllerFactoryBuilder.java
    public double calculateTurnSensorUnitsPosition(double desiredRadians) {
        double currentAngleRadians = m_turnMotor.getSelectedSensorPosition() * motorEncoderPositionCoefficient;
        //return desiredRadians / motorEncoderPositionCoefficient;

        // Reset the NEO's encoder periodically when the module is not rotating.
        // Sometimes (~5% of the time) when we initialize, the absolute encoder isn't
        // fully set up, and we don't
        // end up getting a good reading. If we reset periodically this won't matter
        // anymore.
        // if (m_turnMotor.getSelectedSensorVelocity() * motorEncoderVelocityCoefficient
        // < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
        // if (++resetIteration >= ENCODER_RESET_ITERATIONS) {
        // resetIteration = 0;
        // double absoluteAngle = getAbsoluteAngle();
        // System.out.println("Reseting motors");
        // m_turnMotor.setSelectedSensorPosition(absoluteAngle /
        // motorEncoderPositionCoefficient);
        // currentAngleRadians = absoluteAngle;
        // }
        // } else {
        // resetIteration = 0;
        // }

        double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
        if (currentAngleRadiansMod < 0.0) {
            currentAngleRadiansMod += 2.0 * Math.PI;
        }

        // The reference angle has the range [0, 2pi) but the Falcon's encoder can go
        // above that
        double adjustedReferenceAngleRadians = desiredRadians + currentAngleRadians - currentAngleRadiansMod;
        if (desiredRadians - currentAngleRadiansMod > Math.PI) {
            adjustedReferenceAngleRadians -= 2.0 * Math.PI;
        } else if (desiredRadians - currentAngleRadiansMod < -Math.PI) {
            adjustedReferenceAngleRadians += 2.0 * Math.PI;
        }

        return adjustedReferenceAngleRadians / motorEncoderPositionCoefficient;
    }

    public double calculateDriveSensorUnitsPer100Ms(double speedMetersPerSecond) {
        double driveSensorUnitsPer100Ms = metersToClicks(speedMetersPerSecond) / 10;
        return MathUtil.clamp(driveSensorUnitsPer100Ms, 0, m_sensorUnitsMaxVelocity);
    }

    public double clicksToMeters(double clicks) {
        return (clicks / encoderClicksPerRotation) * kWheelCircumference;
    }

    public double metersToClicks(double meters) {
        return (meters / kWheelCircumference) * encoderClicksPerRotation;
    }

    public double getVelocity() {
        double nativeSpeed = m_driveMotor.getSelectedSensorVelocity();
        return clicksToMeters(nativeSpeed * 10);
    }

    // From SDS library - RADIANS
    public double getAbsoluteAngle() {

        //return Math.toRadians(m_canCoder.getAbsolutePosition());
        double angle = Math.toRadians(m_canCoder.getAbsolutePosition());
        angle %= 2.0 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }

        return angle;
    }

    // From SDS Library
    public double getStateAngle() {
        double motorAngleRadians = m_turnMotor.getSelectedSensorPosition() * motorEncoderPositionCoefficient;
        motorAngleRadians %= 2.0 * Math.PI;
        if (motorAngleRadians < 0.0) {
            motorAngleRadians += 2.0 * Math.PI;
        }

        return motorAngleRadians;
    }

    // Debugging functions
    public double getRotationDegrees() {
        return getAbsoluteAngle();
    }

    public double getDistanceInMeters() {
        return clicksToMeters(m_driveMotor.getSelectedSensorPosition());
    }

    public double getClicks() {
        return m_driveMotor.getSelectedSensorPosition();
    }

    public void rotationTarget(double desiredRadians) {
        SmartDashboard.putNumber("rotation setpoint", calculateTurnSensorUnitsPosition(desiredRadians));
    }

    public void getRotationCurrent() {
        SmartDashboard.putNumber("actual rotation position", m_turnMotor.getSelectedSensorPosition());
    }

    // Following two functions from team 364
    /**
     * Minimize the change in heading the desired swerve module state would require
     * by potentially
     * reversing the direction the wheel spins. Customized from WPILib's version to
     * include placing
     * in appropriate scope for CTRE onboard control.
     *
     * @param desiredState The desired state.
     * @param currentAngle The current module angle.
     */
    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
        double targetSpeed = desiredState.speedMetersPerSecond;
        double delta = targetAngle - currentAngle.getDegrees();
        if (Math.abs(delta) > 90) {
            targetSpeed = -targetSpeed;
            targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
        }
        return new SwerveModuleState(targetSpeed, Rotation2d.fromRadians(targetAngle));
    }

    /**
     * @param scopeReference Current Angle
     * @param newAngle       Target Angle
     * @return Closest angle within scope
     */
    private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }
        return newAngle;
    }
}
