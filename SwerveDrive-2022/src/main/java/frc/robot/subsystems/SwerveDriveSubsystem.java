package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.swervedrivespecialties.swervelib.SwerveModule;
import com.swervedrivespecialties.swervelib.ctre.CanCoderFactoryBuilder.Direction;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// SPS MK4i L2

public class SwerveDriveSubsystem extends SubsystemBase {

    private static SwerveDriveSubsystem instance = null;
    
    public static SwerveDriveSubsystem getInstance() {
        return instance;
    }

    private SwerveDriveKinematics m_kinematics;

    private WPI_Pigeon2 m_gyro;

    private SwerveModule m_frontLeftModule;
    private SwerveModule m_frontRightModule;
    private SwerveModule m_backLeftModule;
    private SwerveModule m_backRightModule;

    private ChassisSpeeds m_chassisSpeeds;

    private Configuration m_config;

    public static class Configuration {
        public double m_maxVoltage = 0;
        public double m_maxMetersPerSecond = 0;
        public double m_maxRadiansPerSecond = 0;
    }

    public SwerveDriveSubsystem(SwerveModule frontLeftModule, SwerveModule frontRightModule, SwerveModule backLeftModule, SwerveModule backRightModule, WPI_Pigeon2 gyro) {


        instance = this;
        
        m_kinematics = new SwerveDriveKinematics(
            new Translation2d(Constants.SWERVE.TRACK_WIDTH / 2.0, Constants.SWERVE.WHEEL_BASE / 2.0),
            new Translation2d(Constants.SWERVE.TRACK_WIDTH / 2.0, -Constants.SWERVE.WHEEL_BASE / 2.0),
            new Translation2d(-Constants.SWERVE.TRACK_WIDTH / 2.0, Constants.SWERVE.WHEEL_BASE / 2.0),
            new Translation2d(-Constants.SWERVE.TRACK_WIDTH / 2.0, -Constants.SWERVE.WHEEL_BASE / 2.0)
        );

        m_gyro = gyro;

        m_frontLeftModule = frontLeftModule;
        m_frontRightModule = frontRightModule;
        m_backLeftModule = backLeftModule;
        m_backRightModule = backRightModule;

    }

    public void configure(Configuration config) {
        m_config = config;
    }

    public void zeroGyro() {
        m_gyro.setYaw(0.0);
    }

    public Rotation2d getGyroRotation() {
        return Rotation2d.fromDegrees(m_gyro.getYaw());
    }

    public ChassisSpeeds getChassisSpeeds(double x, double y, double rotation) {
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            x * m_config.m_maxMetersPerSecond, 
            y * m_config.m_maxMetersPerSecond, 
            rotation * m_config.m_maxMetersPerSecond, 
            getGyroRotation()
        );

        return chassisSpeeds;
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        m_chassisSpeeds = chassisSpeeds;
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, m_config.m_maxMetersPerSecond);

        m_frontLeftModule.set(states[0].speedMetersPerSecond * 0 / m_config.m_maxMetersPerSecond, states[0].angle.getRadians());
        m_frontRightModule.set(states[1].speedMetersPerSecond * 0 / m_config.m_maxMetersPerSecond, states[1].angle.getRadians());
        m_backLeftModule.set(states[2].speedMetersPerSecond * 0 / m_config.m_maxMetersPerSecond, states[2].angle.getRadians());
        m_backRightModule.set(states[3].speedMetersPerSecond * 0 / m_config.m_maxMetersPerSecond, states[3].angle.getRadians());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Front Left Angle", m_frontLeftModule.getSteerAngle());
        SmartDashboard.putNumber("Front Right Angle", m_frontRightModule.getSteerAngle());
        SmartDashboard.putNumber("Back Left Angle", m_backLeftModule.getSteerAngle());
        SmartDashboard.putNumber("Back Right Angle", m_backRightModule.getSteerAngle());
    }
}
