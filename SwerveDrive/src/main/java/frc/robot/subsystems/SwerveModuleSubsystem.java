package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModuleSubsystem extends SubsystemBase {
    private double m_angleOffset;
    private WPI_TalonFX m_angleMotor;
    private WPI_TalonFX m_driveMotor;
    private CANCoder m_angleEncoder;

    public SwerveModuleSubsystem(int angleMotorID, int driveMotorID, int encoderID, double angleOffset, double kP, double kI, double kD, int timeoutMillis) {
        this.m_angleOffset = angleOffset;

        this.m_angleMotor = new WPI_TalonFX(angleMotorID);
        this.m_driveMotor = new WPI_TalonFX(driveMotorID);
        this.m_angleEncoder = new CANCoder(encoderID);

        this.m_angleMotor.config_kP(0, kP, timeoutMillis);
        this.m_angleMotor.config_kI(0, kI, timeoutMillis);
        this.m_angleMotor.config_kD(0, kD, timeoutMillis);
    }
}
