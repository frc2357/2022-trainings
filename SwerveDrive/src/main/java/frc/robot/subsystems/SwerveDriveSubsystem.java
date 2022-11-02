package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// SPS MK4i L2

public class SwerveDriveSubsystem extends SubsystemBase {

    private static SwerveDriveSubsystem instance = null;
    
    public static SwerveDriveSubsystem getInstance() {
        return instance;
    }

    private SwerveDriveKinematics m_kinematics;

    private PigeonIMU m_gyro;

    private SwerveModule m_frontLeftModule;
    private SwerveModule m_frontRightModule;
    private SwerveModule m_backLeftModule;
    private SwerveModule m_backRightModule;

    private ChassisSpeeds m_chassisSpeeds;

    public double m_maxVoltage;
    public double m_maxVelocity;
    public double m_maxAngularVelocity;

    private Configuration m_config;

    public static class Configuration {
        public double m_maxVoltage = 0;
        public double m_maxVelocityMeters = 0;
        public double m_maxAngularVelocity = 0;
    }

    public SwerveDriveSubsystem(SwerveModule frontLeftModule, SwerveModule frontRightModule, SwerveModule backLeftModule, SwerveModule backRightModule, PigeonIMU gyro) {

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

        m_maxVoltage = config.m_maxVoltage;
        m_maxVelocity = config.m_maxVelocityMeters;
        m_maxAngularVelocity = config.m_maxAngularVelocity;
    }

    public void zeroGyro() {
        m_gyro.setFusedHeading(0.0);
    }

    public Rotation2d getGyroRotation() {
        return Rotation2d.fromDegrees(m_gyro.getFusedHeading());
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        m_chassisSpeeds = chassisSpeeds;
    }

    @Override
    public void periodic() {
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, m_maxVelocity);

        m_frontLeftModule.set(states[0].speedMetersPerSecond / m_maxVelocity, states[0].angle.getRadians());
        m_frontRightModule.set(states[1].speedMetersPerSecond / m_maxVelocity, states[1].angle.getRadians());
        m_backLeftModule.set(states[2].speedMetersPerSecond / m_maxVelocity, states[2].angle.getRadians());
        m_backRightModule.set(states[3].speedMetersPerSecond / m_maxVelocity, states[3].angle.getRadians());
    }
}
