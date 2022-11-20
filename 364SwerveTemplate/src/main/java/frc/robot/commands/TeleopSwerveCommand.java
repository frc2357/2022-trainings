package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.Constants;

public class TeleopSwerveCommand extends CommandBase {
    
    private double m_rotation;
    private Translation2d m_translation;
    
    private SwerveDriveSubsystem m_swerveSubsystem;
    private Joystick m_controller;
    private int m_translationAxis;
    private int m_strafeAxis;
    private int m_rotationAxis;

    public TeleopSwerveCommand(SwerveDriveSubsystem swerveSub, Joystick controller, int translationAxis, int strafeAxis, int rotationAxis) {
        m_swerveSubsystem = swerveSub;
        addRequirements(swerveSub);

        m_controller = controller;
        m_translationAxis = translationAxis;
        m_strafeAxis = strafeAxis;
        m_rotationAxis = rotationAxis;
    }

    @Override
    public void execute() {
        double yAxis = -m_controller.getRawAxis(m_translationAxis);
        double xAxis = -m_controller.getRawAxis(m_strafeAxis);
        double rAxis = -m_controller.getRawAxis(m_translationAxis);

        //TODO: Make deadbands

        m_translation = new Translation2d(yAxis, xAxis).times(Constants.SWERVE.MAX_SPEED_METER_PER_SECOND);
        m_rotation = rAxis * Constants.SWERVE.MAX_RADIANS_PER_SECOND;
        m_swerveSubsystem.drive(m_translation, m_rotation);

    }

}
