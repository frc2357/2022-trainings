package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
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
import frc.robot.util.SwerveModuleConstants;

public class SwerveDriveSubsystem extends SubsystemBase {

    private static SwerveDriveSubsystem instance;

    public static SwerveDriveSubsystem getInstance() {
        return instance;
    }

    public static class Configuration {
        public double m_wheelBase = 0.0;
        public double m_trackWidth = 0.0;
        public double m_wheelDiameter = 0.0;
        public double m_wheelCircumference = 0.0;

        public double m_driveGearRatio = 0.0;
        public double m_turnGearRatio = 0.0;

        public SwerveDriveKinematics m_kinematics = null;

        public int m_turnContinuousCurrentLimit = 0;
        public int m_turnPeakCurrentLimit = 0;
        public double m_turnPeakCurrentDuration = 0.0;
        public boolean m_turnEnableCurrentLimit = false;
        public SupplyCurrentLimitConfiguration m_turnCurrentLimitConfig = null;

        public int m_driveContinuousCurrentLimit = 0;
        public int m_drivePeakCurrentLimit = 0;
        public double m_drivePeakCurrentDuration = 0.0;
        public boolean m_driveEnableCurrentLimit = false;
        public SupplyCurrentLimitConfiguration m_driveCurrentLimitConfig = null;

        public double m_turnKP = 0.0;
        public double m_turnKI = 0.0;
        public double m_turnKD = 0.0;
        public double m_turnKF = 0.0;

        public double m_driveKP = 0.0;
        public double m_driveKI = 0.0;
        public double m_driveKD = 0.0;
        public double m_driveKF = 0.0;

        public double m_driveKS = 0.0;
        public double m_driveKV = 0.0;
        public double m_driveKA = 0.0;

        public NeutralMode m_turnNeutralMode = null;
        public NeutralMode m_driveNeutralMode = null;

        public boolean m_driveInvert = false;
        public boolean m_turnInvert = false;
        public boolean m_cancoderInvert = false;
        public boolean m_gyroInvert = false;

        public SwerveModuleConstants m_frontLeftModuleConstants = null;
        public SwerveModuleConstants m_frontRightModuleConstants = null;
        public SwerveModuleConstants m_backLeftModuleConstants = null;
        public SwerveModuleConstants m_backRightModuleConstants = null;

        public TalonFXConfiguration m_turnMotorConfig = null;
        public TalonFXConfiguration m_driveMotorConfig = null;
        public CANCoderConfiguration m_cancoderConfig = null;
    }

    private Configuration m_config;

    public SwerveDriveOdometry m_odometry;

    public SwerveModule[] m_swerveModules;
    
    public PigeonIMU m_gyro;

    public SwerveDriveSubsystem() {
        instance = this;

        m_gyro = new PigeonIMU(Constants.CAN_ID.GYRO);
        m_gyro.configFactoryDefault();
        zeroGyro();

        
    }

    public void configure(Configuration config) {
        m_config = config;

        m_odometry = new SwerveDriveOdometry(m_config.m_kinematics, getYaw());

        m_swerveModules = new SwerveModule[] {
            new SwerveModule(0, m_config.m_frontLeftModuleConstants),
            new SwerveModule(1, m_config.m_frontRightModuleConstants),
            new SwerveModule(2, m_config.m_backLeftModuleConstants),
            new SwerveModule(3, m_config.m_backRightModuleConstants)
        };

        for (SwerveModule mod : m_swerveModules) {
            mod.configure(m_config);
        }
    }

    public void drive(Translation2d translation, double rotation) {
        SwerveModuleState[] swerveModuleStates = 
            m_config.m_kinematics.toSwerveModuleStates(
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
        return (m_config.m_gyroInvert) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
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
