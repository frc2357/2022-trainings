package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDriveCommand extends CommandBase {

    private DriveSubsystem m_driveSub;
    private double m_timeMillis, m_timeTotal, m_turn, m_speed;

    public AutoDriveCommand(DriveSubsystem driveSub, double timeMillis, double speed, double turn) {
        m_driveSub = driveSub;
        m_timeMillis = timeMillis;
        m_turn = turn;
        m_speed = speed;

        addRequirements(m_driveSub);
    }

    @Override
    public void initialize() {
        m_driveSub.driveProportional(m_speed, m_turn);
        m_timeTotal = System.currentTimeMillis() + m_timeMillis;
    }

    @Override
    public boolean isFinished() {
        return m_timeTotal < System.currentTimeMillis();
    }

    @Override
    public void end(boolean interrupted) {
        m_driveSub.driveProportional(0.0, 0.0);
    }
}