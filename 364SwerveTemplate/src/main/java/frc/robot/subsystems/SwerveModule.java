package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.CTREModuleState;
import frc.robot.util.Conversions;
import frc.robot.util.SwerveModuleConstants;

public class SwerveModule {
    public int m_moduleNumber;
    private double m_angleOffset;
    private WPI_TalonFX m_turnMotor;
    private WPI_TalonFX m_driveMotor;
    private CANCoder m_cancoder;
    private double m_lastAngle;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.SWERVE.DRIVE_KS, Constants.SWERVE.DRIVE_KV, Constants.SWERVE.DRIVE_KA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        m_moduleNumber = moduleNumber;
        m_angleOffset = moduleConstants.angleOffset;

        m_cancoder = new CANCoder(moduleConstants.cancoderID);
        configCANCoder();

        m_turnMotor = new WPI_TalonFX(moduleConstants.turnMotorID);
        configTurnMotor();

        m_driveMotor = new WPI_TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();

        m_lastAngle = getState().angle.getDegrees();
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);

        double percentOutput = desiredState.speedMetersPerSecond / Constants.SWERVE.MAX_SPEED_METER_PER_SECOND;
        m_driveMotor.set(ControlMode.PercentOutput, percentOutput);

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SWERVE.MAX_SPEED_METER_PER_SECOND * 0.01)) ? m_lastAngle : desiredState.angle.getDegrees(); // Prevents jittering
        m_turnMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle, Constants.SWERVE.TURN_GEAR_RATIO));
        m_lastAngle = angle;
    }

    private void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - m_angleOffset, Constants.SWERVE.TURN_GEAR_RATIO);
        m_turnMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configCANCoder() {
        m_cancoder.configFactoryDefault();
        m_cancoder.configAllSettings(Robot.ctreConfigs.swerveCANCoderConfig);
    }

    private void configTurnMotor() {
        m_turnMotor.configFactoryDefault();
        m_turnMotor.configAllSettings(Robot.ctreConfigs.swerveTurnFXConfig);
        m_turnMotor.setInverted(Constants.SWERVE.TURN_INVERT);
        m_turnMotor.setNeutralMode(Constants.SWERVE.TURN_NEUTRAL_MODE);
        resetToAbsolute();
    }

    private void configDriveMotor() {

    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(m_cancoder.getAbsolutePosition());
    }

    public SwerveModuleState getState() {
        double velocity = Conversions.falconToMPS(m_driveMotor.getSelectedSensorVelocity(), Constants.SWERVE.WHEEL_CIRCUMFERENCE, Constants.SWERVE.DRIVE_GEAR_RATIO);
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(m_turnMotor.getSelectedSensorPosition(), Constants.SWERVE.TURN_GEAR_RATIO));
        return new SwerveModuleState(velocity, angle);
    }
}
