// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
// import frc.lib.trajectory.PathPlannerTrajectory;
import frc.robot.Constants;

import java.util.function.Supplier;

public class DrivetrainSubsystem extends SubsystemBase {

  private static DrivetrainSubsystem m_instance;

  public static DrivetrainSubsystem getInstance() {
    return m_instance;
  }

  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is
   * useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 10.0;

  // The formula for calculating the theoretical maximum velocity is:
  // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
  // pi
  // By default this value is setup for a Mk3 standard module using Falcon500s to
  // drive.
  // An example of this constant for a Mk4 L2 module with NEOs to drive is:
  // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
  // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight
   * line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
      SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
      SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;

  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also
  // replace this with a measured amount.

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      // Front left
      new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Front right
      new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back left
      new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back right
      new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0));

  private Configuration m_config;
  // By default we use a Pigeon for our gyroscope. But if you use another
  // gyroscope, like a NavX, you can change this.
  // The important thing about how you configure your gyroscope is that rotating
  // the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
  // FIXME Remove if you are using a Pigeon
  private final WPI_Pigeon2 m_pigeon = new WPI_Pigeon2(Constants.DRIVETRAIN_PIGEON_ID);

  private final WPI_TalonFX m_frontLeftDriveMotor;
  private final WPI_TalonFX m_frontRightDriveMotor;
  private final WPI_TalonFX m_backLeftDriveMotor;
  private final WPI_TalonFX m_backRightDriveMotor;

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  private SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, getHeading());

  private DifferentialDriveOdometry m_tankOdometry;

  private static PIDController m_xController;
  private static PIDController m_yController;
  private static PIDController m_rotationController;

  public static class Configuration {
    public double m_maxVoltage = 0;
    public double m_maxMetersPerSecond = 0;
    public double m_maxRadiansPerSecond = 0;
  }

  public DrivetrainSubsystem(SwerveModule frontRightModule, SwerveModule backRightModule, SwerveModule frontLeftModule,
      SwerveModule backLeftModule, WPI_Pigeon2 pigeon2) {
    m_instance = this;

    m_frontLeftDriveMotor = new WPI_TalonFX(Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR);
    m_frontRightDriveMotor = new WPI_TalonFX(Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR);
    m_backLeftDriveMotor = new WPI_TalonFX(Constants.BACK_LEFT_MODULE_DRIVE_MOTOR);
    m_backRightDriveMotor = new WPI_TalonFX(Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR);

    m_frontLeftModule = frontLeftModule;

    m_frontRightModule = frontRightModule;

    m_backLeftModule = backLeftModule;

    m_backRightModule = backRightModule;

    m_backLeftDriveMotor.follow(m_frontLeftDriveMotor);
    m_backRightDriveMotor.follow(m_frontRightDriveMotor);

    m_tankOdometry = new DifferentialDriveOdometry(getHeading());
    resetEncoders();
    m_tankOdometry.resetPosition(new Pose2d(0, 0, new Rotation2d(0)), getHeading());
  }

  public void Config(Configuration config) {
    m_config = config;

    m_xController = new PIDController(Constants.DRIVE.DRIVETRAIN_PX_CONTROLLER, 0, 0);
    m_yController = new PIDController(Constants.DRIVE.DRIVETRAIN_PY_CONTROLLER, 0, 0);
    // m_rotationController = new ProfiledPIDController(Constants.DRIVE.DRIVETRAIN_PTHETA_CONTROLLER, 0, 0, new TrapezoidProfile.Constraints(
    //   Constants.DRIVE.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
    //   Constants.DRIVE.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED
    // ));
    m_rotationController = new PIDController(Constants.DRIVE.DRIVETRAIN_PTHETA_CONTROLLER, 0, 0);
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the
   * robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope() {
    m_pigeon.reset();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, getHeading());
  }

  private Rotation2d getHeading() {
    double x = 360 - m_pigeon.getYaw();
    return Rotation2d.fromDegrees(x);
  }

  private void resetEncoders() {
    //TODO: Figure this out
  }

  public Rotation2d getGyroscopeRotation() {
    return Rotation2d.fromDegrees(m_pigeon.getYaw());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }

  public void setTankDriveVolts(double leftVolts, double rightVolts) {
    m_frontLeftModule.set(leftVolts, 0);
    m_frontRightModule.set(rightVolts, 0);
    m_backLeftModule.set(leftVolts, 0);
    m_backRightModule.set(rightVolts, 0);
  }

  @Override
  public void periodic() {
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

    m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        states[0].angle.getRadians());
    m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        states[1].angle.getRadians());
    // m_backLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
    //     states[2].angle.getRadians());
    // m_backRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
    //     states[3].angle.getRadians());

    SmartDashboard.putNumber("Angle", m_pigeon.getYaw());
    
    m_odometry.update(getHeading(), states);
    m_tankOdometry.update(getHeading(), getLeftDistance(), getLeftDistance());

    SmartDashboard.putNumber("Pose Meters X", m_tankOdometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Pose Meters Y", m_tankOdometry.getPoseMeters().getY());

  }

  // public SequentialCommandGroup followPathCommand(final boolean shouldResetOdometry, String trajectoryFileName) {
  //   final PathPlannerTrajectory trajectory = PathPlanner.loadPath(trajectoryFileName, 3.5, 3);

  //   return new InstantCommand(() -> {
  //     if (shouldResetOdometry) {
  //       PathPlannerState initialSample = (PathPlannerState) trajectory.sample(0);
  //       Pose2d initialPose = new Pose2d(initialSample.poseMeters.getTranslation(), initialSample.holonomicRotation);
  //       m_odometry.resetPosition(initialPose, getHeading());
  //     }
  //   //TODO: Create xController, yController, and thetaController
  //   }).andThen(new PPSwerveControllerCommand(
  //     trajectory,
  //     () -> getPose(),
  //     m_kinematics,
  //     m_xController,
  //     m_yController,
  //     m_rotationController,
  //     (SwerveModuleState[] moduleStates) -> {
  //       drive(m_kinematics.toChassisSpeeds(moduleStates));
  //     },
  //     (Subsystem)this
  //   )).andThen(() -> drive(new ChassisSpeeds()), this);
  // }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getVelocityLeftEncoder(), getVelocityRightEncoder());
  }

  public double getVelocityLeftEncoder() {
    double nativeSpeed = m_frontLeftDriveMotor.getSelectedSensorVelocity();
    return clicksToMeters(nativeSpeed * 10);
  }

/**
 * 
 * @return The speed of the drive in meters per second
 */
  public double getVelocityRightEncoder() {
    double nativeSpeed = m_frontRightDriveMotor.getSelectedSensorVelocity();
    return clicksToMeters(nativeSpeed * 10);
  }

  public double clicksToMeters(double clicks) {
    return (clicks / Constants.DRIVE.ENCODER_CLICKS_PER_ROTATION) * Constants.WHEEL_CIRCUMFERENCE;
  }

  public double getLeftDistance() {
    return clicksToMeters(m_frontLeftDriveMotor.getSelectedSensorPosition());
  }

  public double getRightDistance() {
      return clicksToMeters(m_frontRightDriveMotor.getSelectedSensorPosition());
  }

}
