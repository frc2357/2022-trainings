package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class AutoCommandGroupDrive extends SequentialCommandGroup{
    public AutoCommandGroupDrive(DriveSubsystem dS){
        addCommands(new AutoDriveCommand(dS, 1, 1, 200));
        addCommands(new AutoDriveCommand(dS, -1, 1, 200));
        addCommands(new AutoDriveCommand(dS, 0.1, 0.1, 200));
    }
    
}
