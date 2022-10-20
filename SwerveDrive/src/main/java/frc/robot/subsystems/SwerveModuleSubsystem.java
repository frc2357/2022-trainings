package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.CTREModuleState;
import frc.robot.util.Conversions;
import frc.robot.util.SwerveModuleConstants;

public class SwerveModuleSubsystem extends SubsystemBase {
    private final int m_moduleNumber;
    private double m_angleOffset;
    private WPI_TalonFX m_angleMotor;
    private WPI_TalonFX m_driveMotor;
    private CANCoder m_angleEncoder;

    private double m_anglekP;
    private double m_anglekI;
    private double m_anglekD;
    private double m_anglekF;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.SWERVE_CONSTANTS.DRIVE_KS, Constants.SWERVE_CONSTANTS.DRIVE_KV, Constants.SWERVE_CONSTANTS.DRIVE_KA);

    public SwerveModuleSubsystem(int moduleNumber, SwerveModuleConstants constants) {
        this.m_moduleNumber = moduleNumber;

        this.m_angleOffset = constants.ANGLE_OFFSET;

        this.m_driveMotor = new WPI_TalonFX(constants.DRIVE_MOTOR_ID);
        this.m_angleMotor = new WPI_TalonFX(constants.ANGLE_MOTOR_ID);
        this.m_angleEncoder = new CANCoder(constants.CANCODER_ID);
        
        this.setAnglePID(Constants.SWERVE_CONSTANTS.ANGLE_KP, Constants.SWERVE_CONSTANTS.ANGLE_KI, Constants.SWERVE_CONSTANTS.ANGLE_KD);
    }
    
    public void setState(SwerveModuleState state, boolean isOpenLoop) {
        state = CTREModuleState.optimize(state, getState().angle);

        if (isOpenLoop) {
            double percentOutput = state.speedMetersPerSecond / Constants.SWERVE_CONSTANTS.MAX_SPEED;
            this.m_driveMotor.set(ControlMode.PercentOutput, percentOutput);
        } else {
            double velocity = Conversions.MPSToFalcon(state.speedMetersPerSecond, Constants.SWERVE_CONSTANTS.WHEEL_CIRCUMFERENCE, Constants.SWERVE_CONSTANTS.DRIVE_GEAR_RATIO);
            m_driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(state.speedMetersPerSecond));
        }
    }

    private Object getState() {
        double velocity = Conversions.falconToMPS(this.m_driveMotor.getSelectedSensorVelocity(), Constants.SWERVE_CONSTANTS.WHEEL_CIRCUMFERENCE, Constants.SWERVE_CONSTANTS.DRIVE_GEAR_RATIO);
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(this.m_angleMotor.getSelectedSensorPosition(), Constants.SWERVE_CONSTANTS.ANGLE_GEAR_RATIO));
        return new SwerveModuleState(velocity, angle);
    }

    public void setAnglePID(double kP, double kI, double kD) {
        this.m_anglekP = kP;
        this.m_anglekP = kI;
        this.m_anglekP = kD;

        this.m_angleMotor.config_kP(0, kP, Constants.SWERVE_CONSTANTS.TIMEOUT_MILLIS);
        this.m_angleMotor.config_kP(0, kI, Constants.SWERVE_CONSTANTS.TIMEOUT_MILLIS);
        this.m_angleMotor.config_kP(0, kD, Constants.SWERVE_CONSTANTS.TIMEOUT_MILLIS);
    }
}
