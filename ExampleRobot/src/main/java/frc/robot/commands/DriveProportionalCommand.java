package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveProportionalCommand extends CommandBase {
    private DriveSubsystem m_driveSub;
    private XboxController m_controller;

    public DriveProportionalCommand(DriveSubsystem driveSub, XboxController controller) {
        m_driveSub = driveSub;
        m_controller = controller;
    }

    @Override 
    public void execute() {
        m_driveSub.driveProportionalWithStick(m_controller.getLeftY(), m_controller.getRightX());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
