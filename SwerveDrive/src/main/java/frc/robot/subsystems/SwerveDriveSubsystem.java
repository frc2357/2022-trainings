package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDriveSubsystem extends SubsystemBase {
    private static SwerveDriveSubsystem instance = null;

    public static SwerveDriveSubsystem getInstance() {
        return instance;
    }

    public static class Configuration {

    }

    private SwerveModuleSubsystem[] m_swerveModules;
    private SwerveDriveOdometry m_swerveOdometry;

    private PigeonIMU m_pigeon;
    private boolean m_isGyroReversed;

    private Configuration m_config;

    public SwerveDriveSubsystem() {
        m_swerveOdometry = new SwerveDriveOdometry(Constants.SWERVE_CONSTANTS.SWERVE_KINEMATICS, Rotation2d.fromDegrees(m_pigeon.getYaw()));

        m_swerveModules = new SwerveModuleSubsystem[] {
            new SwerveModuleSubsystem(0, Constants.SWERVE_CONSTANTS.MOD0),
            new SwerveModuleSubsystem(1, Constants.SWERVE_CONSTANTS.MOD1),
            new SwerveModuleSubsystem(2, Constants.SWERVE_CONSTANTS.MOD2),
            new SwerveModuleSubsystem(3, Constants.SWERVE_CONSTANTS.MOD3)
        };
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] moduleStates;

        moduleStates = Constants.SWERVE_CONSTANTS.SWERVE_KINEMATICS.toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                rotation,
                Rotation2d.fromDegrees(m_pigeon.getYaw())
            )
            : new ChassisSpeeds(
                translation.getX(),
                translation.getY(),
                rotation
            )
        );

        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.SWERVE_CONSTANTS.MAX_SPEED);

        for (SwerveModuleSubsystem mod : m_swerveModules) {
            mod.setState(moduleStates[mod.getModuleNumber()], isOpenLoop);
        }
    }

    public Pose2d getPose() {
        return m_swerveOdometry.getPoseMeters();
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModuleSubsystem mod : m_swerveModules) {
            states[mod.getModuleNumber()] = mod.getState();
        }
        return states;
    }

    public void setBrake() {
        for (SwerveModuleSubsystem mod : m_swerveModules) {
            mod.setBrake();
        }
    }

    public void setCoast() {
        for (SwerveModuleSubsystem mod : m_swerveModules) {
            mod.setCoast();
        }
    }
}
