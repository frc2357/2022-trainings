package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controls.SwerveDriveControls;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class DefaultDriveCommand extends CommandBase {
    private final SwerveDriveSubsystem m_drivetrainSubsystem;
    private final SwerveDriveControls m_controls;

    public DefaultDriveCommand(SwerveDriveSubsystem drivetrainSubsystem,
                               SwerveDriveControls controls) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_controls = controls;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        m_drivetrainSubsystem.drive(
                m_controls.getX(),
                m_controls.getY(),
                m_controls.getRotation()
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
