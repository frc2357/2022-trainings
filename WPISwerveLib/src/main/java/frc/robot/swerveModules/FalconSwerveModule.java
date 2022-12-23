package frc.robot.swerveModules;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.Drivetrain;

public class FalconSwerveModule {
    private static final double gearRatio = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    private static final double kWheelDiameter = 0.10033;
    private static final double kWheelCircumference = kWheelDiameter * Math.PI;
    private static final double encoderClicksPerRotation = 2048 * (1/gearRatio);

    private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
    private static final double kModuleMaxAngularAcceleration =
        2 * Math.PI; // radians per second squared
  
    private final WPI_TalonFX m_driveMotor;
    private final WPI_TalonFX m_turnMotor;
  
    private final CANCoder m_canCoder;
  
    // Gains are for example purposes only - must be determined for your own robot!
    private final PIDController m_drivePIDController = new PIDController(0.56122, 0, 0);
  
    // Gains are for example purposes only - must be determined for your own robot!
    private final ProfiledPIDController m_turningPIDController =
        new ProfiledPIDController(
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
        int canCoderID
    ) {
        m_driveMotor = new WPI_TalonFX(driveMotorID);
        m_turnMotor = new WPI_TalonFX(turnMotorID);
        m_canCoder = new CANCoder(canCoderID);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            getVelocity(), Rotation2d.fromDegrees(m_canCoder.getAbsolutePosition())
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            clicksToMeters(m_driveMotor.getSelectedSensorPosition()), Rotation2d.fromDegrees(m_canCoder.getAbsolutePosition())
        );
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(m_canCoder.getAbsolutePosition()));

        final double driveOutput = m_drivePIDController.calculate(getVelocity(), state.speedMetersPerSecond);
        final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

        final double turnOutput = m_turningPIDController.calculate(Math.toRadians(m_canCoder.getAbsolutePosition()), state.angle.getRadians());
        final double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

        m_driveMotor.setVoltage(driveOutput + driveFeedforward);
        m_turnMotor.setVoltage(turnOutput + turnFeedforward);
    }

    public double clicksToMeters(double clicks) {
        return (clicks / encoderClicksPerRotation) * kWheelCircumference;
    }

    public double getVelocity() {
        double nativeSpeed = m_driveMotor.getSelectedSensorVelocity();
        return clicksToMeters(nativeSpeed * 10);
    }

    // Debugging functions
    public double getRotationDegrees() {
        return m_canCoder.getAbsolutePosition();
    }

    public double getDistanceInMeters() {
        return clicksToMeters(m_driveMotor.getSelectedSensorPosition());
    }

    public double getClicks() {
        return m_driveMotor.getSelectedSensorPosition();
    }
}
