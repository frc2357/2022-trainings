package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase{
    private DriveSubsystem dSubsystem;
    private XboxController xController;
    public DriveCommand(DriveSubsystem dS,XboxController xC){
        dSubsystem=dS;
        xController=xC;
        addRequirements(dS);
    }
    @Override
    public void execute(){
        dSubsystem.driveProportional(xController.getLeftX(), xController.getRightX());
    }
    @Override
    public boolean isFinished(){
        return false;
    }

}
