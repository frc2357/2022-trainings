// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ExampleGlobalMeasurementSensor;
import frc.robot.swerveModules.FalconSwerveModule;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final FalconSwerveModule m_frontLeft = new FalconSwerveModule(Constants.CAN_ID.FRONT_LEFT_MODULE_DRIVE_MOTOR,
      Constants.CAN_ID.FRONT_LEFT_MODULE_STEER_MOTOR, Constants.CAN_ID.FRONT_LEFT_MODULE_STEER_ENCODER, Constants.SWERVE.FRONT_LEFT_MODULE_STEER_OFFSET);
  private final FalconSwerveModule m_frontRight = new FalconSwerveModule(
      Constants.CAN_ID.FRONT_RIGHT_MODULE_DRIVE_MOTOR, Constants.CAN_ID.FRONT_RIGHT_MODULE_STEER_MOTOR,
      Constants.CAN_ID.FRONT_RIGHT_MODULE_STEER_ENCODER, Constants.SWERVE.FRONT_RIGHT_MODULE_STEER_OFFSET);
  private final FalconSwerveModule m_backLeft = new FalconSwerveModule(Constants.CAN_ID.BACK_LEFT_MODULE_DRIVE_MOTOR,
      Constants.CAN_ID.BACK_LEFT_MODULE_STEER_MOTOR, Constants.CAN_ID.BACK_LEFT_MODULE_STEER_ENCODER, Constants.SWERVE.BACK_LEFT_MODULE_STEER_OFFSET);
  private final FalconSwerveModule m_backRight = new FalconSwerveModule(Constants.CAN_ID.BACK_RIGHT_MODULE_DRIVE_MOTOR,
      Constants.CAN_ID.BACK_RIGHT_MODULE_STEER_MOTOR, Constants.CAN_ID.BACK_RIGHT_MODULE_STEER_ENCODER, Constants.SWERVE.BACK_RIGHT_MODULE_STEER_OFFSET);

  private final WPI_Pigeon2 m_gyro = new WPI_Pigeon2(5);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    // Front left
    new Translation2d(Constants.SWERVE.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.SWERVE.DRIVETRAIN_WHEELBASE_METERS / 2.0),
    // Front right
    new Translation2d(Constants.SWERVE.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.SWERVE.DRIVETRAIN_WHEELBASE_METERS / 2.0),
    // Back left
    new Translation2d(-Constants.SWERVE.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.SWERVE.DRIVETRAIN_WHEELBASE_METERS / 2.0),
    // Back right
    new Translation2d(-Constants.SWERVE.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.SWERVE.DRIVETRAIN_WHEELBASE_METERS / 2.0));

  /*
   * Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings.
   * The numbers used
   * below are robot specific, and should be tuned.
   */
  private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
      m_kinematics,
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
      },
      new Pose2d(),
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

  public Drivetrain() {
    m_gyro.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);
    SmartDashboard.putNumber("rotation", rot);
    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

    SmartDashboard.putString("Front left desired state", swerveModuleStates[0].toString());
    m_frontLeft.rotationTarget(swerveModuleStates[0].angle.getRadians());

    //System.out.println("Front Left: ");
    //System.out.println(swerveModuleStates[0]);

    // System.out.println("Front Right: ");
    // System.out.println(swerveModuleStates[1]);

    // System.out.println("Back Left: ");
    // System.out.println(swerveModuleStates[2]);

    // System.out.println("Back Right: ");
    // System.out.println(swerveModuleStates[3]);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    //m_frontRight.setDesiredState(swerveModuleStates[1]);
    //m_backLeft.setDesiredState(swerveModuleStates[2]);
    //m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_poseEstimator.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        });

    // Also apply vision measurements. We use 0.3 seconds in the past as an example
    // -- on
    // a real robot, this must be calculated based either on latency or timestamps.
    m_poseEstimator.addVisionMeasurement(
        ExampleGlobalMeasurementSensor.getEstimatedGlobalPose(
            m_poseEstimator.getEstimatedPosition()),
        Timer.getFPGATimestamp() - 0.3);
  }

  @Override
  public void periodic() {
    updateOdometry();

    SmartDashboard.putNumber("front Left Rot", m_frontLeft.getRotationDegrees());
    SmartDashboard.putNumber("front left meters per second", m_frontLeft.getVelocity());
    m_frontLeft.getRotationCurrent();

    // SmartDashboard.putNumber("front right Rot", m_frontRight.getRotationDegrees());
    // SmartDashboard.putNumber("front right meters per second", m_frontRight.getVelocity());

    // SmartDashboard.putNumber("Back Left Rot", m_backLeft.getRotationDegrees());
    // SmartDashboard.putNumber("Back left meters per second", m_backLeft.getVelocity());

    // SmartDashboard.putNumber("Back right Rot", m_backRight.getRotationDegrees());
    // SmartDashboard.putNumber("Back right meters per second", m_backRight.getVelocity());
  }
}
