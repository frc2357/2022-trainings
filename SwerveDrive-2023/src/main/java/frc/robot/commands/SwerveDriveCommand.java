package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controls.DriveControls;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class SwerveDriveCommand extends CommandBase {
    
    private SwerveDriveSubsystem m_driveSubsystem;
    private DriveControls m_driverController;

    public SwerveDriveCommand(SwerveDriveSubsystem driveSubsystem, DriveControls driverController) {
        m_driveSubsystem = driveSubsystem;
        m_driverController = driverController;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        ChassisSpeeds chassisSpeeds = m_driveSubsystem.getChassisSpeeds(m_driverController.getX(), m_driverController.getY(), m_driverController.getRotation());

        m_driveSubsystem.drive(chassisSpeeds);

    }

    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

}
