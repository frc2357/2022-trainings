// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
  private final WPI_TalonFX m_driveMotor;
  private final WPI_TalonFX m_turningMotor;

  private final CANCoder m_driveEncoder;
  private final CANCoder m_turningEncoder;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel The channel of the drive motor.
   * @param turningMotorChannel The channel of the turning motor.
   * @param driveEncoderChannels The channels of the drive encoder.
   * @param turningEncoderChannels The channels of the turning encoder.
   * @param driveEncoderReversed Whether the drive encoder is reversed.
   * @param turningEncoderReversed Whether the turning encoder is reversed.
   */
  public SwerveModule(
      int driveMotorID,
      int turningMotorID,
      int[] driveEncoderChannels,
      int[] turningEncoderChannels,
      boolean driveEncoderReversed,
      boolean turningEncoderReversed) {



    m_driveMotor = new WPI_TalonFX(driveMotorID);
    m_turningMotor = new WPI_TalonFX(turningMotorID);

    m_driveEncoder = new CANCoder(driveMotorID);

    m_turningEncoder = new CANCoder(turningMotorID);

    m_turningMotor.config_kP(0, ModuleConstants.kPModuleDriveController);
    m_turningMotor.config_kI(0, ModuleConstants.kIModuleDriveController);
    m_turningMotor.config_kD(0, ModuleConstants.kDModuleDriveController);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    CANCoderConfiguration driveConfig = new CANCoderConfiguration();
    driveConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    // driveConfig.magnetOffsetDegrees = Math.toDegrees(configuration.getOffset());
    driveConfig.sensorDirection = driveEncoderReversed;

    m_driveEncoder.configAllSettings(driveConfig);


    // Set whether drive encoder should be reversed or not
    // m_driveEncoder.setReverseDirection(driveEncoderReversed);

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    // m_turningEncoder.setDistancePerPulse(ModuleConstants.kTurningEncoderDistancePerPulse);

    // Set whether turning encoder should be reversed or not
    // m_turningEncoder.setReverseDirection(turningEncoderReversed);

    m_driveMotor.config_kP(0, 0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveEncoder.getRate(), new Rotation2d(m_turningEncoder.get()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getDistance(), new Rotation2d(m_turningEncoder.get()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.get()));

    double driveSensorUnitsPer100Ms = state.speedMetersPerSecond * Constants.DriveConstants.ENCODER_CLICKS_PER_METER / Constants.DriveConstants.DRIVE_VELOCITY_FACTOR;
    double turnSensorUnitsPer100Ms = state.angle.getRadians() * Constants.DriveConstants.ENCODER_CLICKS_PER_RADIAN / Constants.DriveConstants.DRIVE_VELOCITY_FACTOR;

    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.set(TalonFXControlMode.Velocity, driveSensorUnitsPer100Ms);
    m_turningMotor.set(TalonFXControlMode.Velocity, turnSensorUnitsPer100Ms);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveEncoder.reset();
    m_turningEncoder.reset();
  }
}
