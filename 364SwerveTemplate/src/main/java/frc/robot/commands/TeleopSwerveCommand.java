package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.Constants;
import frc.robot.controls.SwerveDriveControls;

public class TeleopSwerveCommand extends CommandBase {
    
    private double m_rotation;
    private Translation2d m_translation;
    
    private SwerveDriveSubsystem m_swerveSubsystem;
    private SwerveDriveControls m_controller;

    public TeleopSwerveCommand(SwerveDriveSubsystem swerveSub, SwerveDriveControls controller) {
        m_swerveSubsystem = swerveSub;
        addRequirements(swerveSub);

        m_controller = controller;
    }

    @Override
    public void execute() {
        double yAxis = -m_controller.getTranslation();
        double xAxis = -m_controller.getStrafe();
        double rAxis = -m_controller.getRotation();

        m_translation = new Translation2d(yAxis, xAxis).times(Constants.SWERVE.MAX_SPEED_METER_PER_SECOND);
        m_rotation = rAxis * Constants.SWERVE.MAX_RADIANS_PER_SECOND;
        m_swerveSubsystem.drive(m_translation, m_rotation);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
