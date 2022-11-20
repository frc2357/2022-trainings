// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.ModuleConfiguration;
import com.swervedrivespecialties.swervelib.ctre.CtreUtils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.controls.DriveControls;
import frc.robot.subsystems.SubsystemFactory;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final DriveControls m_driverControls;

  public RobotContainer() {
    // final double TICKS_PER_ROTATION = 2048.0;
    // ModuleConfiguration moduleConfiguration = Mk4SwerveModuleHelper.GearRatio.L2.getConfiguration();

    // TalonFX driveFrontLeft = new TalonFX(Constants.CAN_ID.FRONT_LEFT_DRIVE);
    // TalonFX turnFrontLeft = new TalonFX(Constants.CAN_ID.FRONT_LEFT_ANGLE);
    // TalonFX driveFrontRight = new TalonFX(Constants.CAN_ID.FRONT_RIGHT_DRIVE);
    // TalonFX turnFrontRight = new TalonFX(Constants.CAN_ID.FRONT_RIGHT_ANGLE);
    // TalonFX driveBackLeft = new TalonFX(Constants.CAN_ID.BACK_LEFT_DRIVE);
    // TalonFX turnBackLeft = new TalonFX(Constants.CAN_ID.BACK_LEFT_ANGLE);
    // TalonFX driveBackRight = new TalonFX(Constants.CAN_ID.BACK_RIGHT_DRIVE);
    // TalonFX turnBackRight = new TalonFX(Constants.CAN_ID.BACK_RIGHT_ANGLE);

    // TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

    // double sensorPositionCoefficient = Math.PI * moduleConfiguration.getWheelDiameter() * moduleConfiguration.getDriveReduction() / TICKS_PER_ROTATION;
    // double sensorVelocityCoefficient = sensorPositionCoefficient * 10.0;

    // if (hasVoltageCompensation()) {
    //     motorConfiguration.voltageCompSaturation = nominalVoltage;
    // }

    // if (hasCurrentLimit()) {
    //     motorConfiguration.supplyCurrLimit.currentLimit = currentLimit;
    //     motorConfiguration.supplyCurrLimit.enable = true;
    // }

    // // CtreUtils.checkCtreError(driveFrontLeft.configAllSettings(motorConfiguration), "Failed to configure DriveFrontLeft");
    // CtreUtils.checkCtreError(driveFrontLeft.configAllSettings(motorConfiguration), "Failed to configure TurnFrontLeft");
    // CtreUtils.checkCtreError(driveFrontLeft.configAllSettings(motorConfiguration), "Failed to configure DriveFrontRight");
    // CtreUtils.checkCtreError(driveFrontLeft.configAllSettings(motorConfiguration), "Failed to configure TurnFrontRight");
    // CtreUtils.checkCtreError(driveFrontLeft.configAllSettings(motorConfiguration), "Failed to configure DriveBackLeft");
    // CtreUtils.checkCtreError(driveFrontLeft.configAllSettings(motorConfiguration), "Failed to configure TurnBackLeft");
    // CtreUtils.checkCtreError(driveFrontLeft.configAllSettings(motorConfiguration), "Failed to configure DriveBackRight");
    // CtreUtils.checkCtreError(driveFrontLeft.configAllSettings(motorConfiguration), "Failed to configure TurnBackRight");
    

    SubsystemFactory subsystemFactory = new SubsystemFactory();
    SwerveDriveSubsystem driveSubsystem = subsystemFactory.CreateSwerveDriveSubsystem();

    XboxController driverController = new XboxController(Constants.CONTROLLER.DRIVE_CONTROLLER_PORT);
    m_driverControls = new DriveControls(driverController, Constants.CONTROLLER.DRIVE_CONTROLLER_DEADBAND);

    driveSubsystem.setDefaultCommand(new SwerveDriveCommand(driveSubsystem, m_driverControls));

    // Configure the button bindings
    configureButtonBindings();
  }

  private double nominalVoltage = Double.NaN;
  private double currentLimit = Double.NaN;

  public boolean hasVoltageCompensation() {
    return Double.isFinite(nominalVoltage);
  }

  public boolean hasCurrentLimit() {
    return Double.isFinite(currentLimit);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
