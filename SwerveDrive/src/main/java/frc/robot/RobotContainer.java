// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    SubsystemFactory subsystemFactory = new SubsystemFactory();
    SwerveDriveSubsystem driveSubsystem = subsystemFactory.CreateSwerveDriveSubsystem();

    XboxController driverController = new XboxController(Constants.CONTROLLER.DRIVE_CONTROLLER_PORT);
    m_driverControls = new DriveControls(driverController, Constants.CONTROLLER.DRIVE_CONTROLLER_DEADBAND);

    driveSubsystem.setDefaultCommand(new SwerveDriveCommand(driveSubsystem, m_driverControls));

    // Configure the button bindings
    configureButtonBindings();
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
