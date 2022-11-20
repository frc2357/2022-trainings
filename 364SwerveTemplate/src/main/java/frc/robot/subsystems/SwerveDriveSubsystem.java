package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDriveSubsystem extends SubsystemBase {
    public SwerveDriveOdometry m_odometry;

    public SwerveModule[] m_swerveModules;
    
    public PigeonIMU m_gyro;

    public SwerveDriveSubsystem() {
        m_gyro = new PigeonIMU(Constants.CAN_ID.GYRO);
        m_gyro.configFactoryDefault();
        zeroGyro();

        m_odometry = new SwerveDriveOdometry(Constants.SWERVE.SWERVE_KINEMATICS, getYaw());

        m_swerveModules = new SwerveModule[] {
            new SwerveModule(0, Constants.SWERVE.FRONT_LEFT_MODULE.constants),
            new SwerveModule(1, Constants.SWERVE.FRONT_RIGHT_MODULE.constants),
            new SwerveModule(2, Constants.SWERVE.BACK_LEFT_MODULE.constants),
            new SwerveModule(3, Constants.SWERVE.BACK_RIGHT_MODULE.constants)
        };
    }

    public void drive(Translation2d translation, double rotation) {
        SwerveModuleState[] swerveModuleStates = 
            Constants.SWERVE.SWERVE_KINEMATICS.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw())
            );

        setModuleStates(swerveModuleStates);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SWERVE.MAX_SPEED_METER_PER_SECOND);

        for (SwerveModule mod : m_swerveModules) {
            mod.setDesiredState(desiredStates[mod.m_moduleNumber]);
        }
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(pose, getYaw());
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];

        for(SwerveModule mod : m_swerveModules) {
            states[mod.m_moduleNumber] = mod.getState();
        }
        return states;
    }

    public void zeroGyro() {
        m_gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        double[] ypr = new double[3];
        m_gyro.getYawPitchRoll(ypr);
        return (Constants.SWERVE.INVERT_GYRO) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
    }

    @Override
    public void periodic() {
        m_odometry.update(getYaw(), getStates());

        for (SwerveModule mod: m_swerveModules) {
            SmartDashboard.putNumber("Module " + mod.m_moduleNumber + " CurrentAngle", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Module " + mod.m_moduleNumber + " TargetAngle", mod.getState().angle.getDegrees());
            SmartDashboard.putNumber("Module " + mod.m_moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }
    }
}
