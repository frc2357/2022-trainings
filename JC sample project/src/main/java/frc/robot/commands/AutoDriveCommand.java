package frc.robot.commands;

import java.util.Date;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDriveCommand extends CommandBase{
    private DriveSubsystem dS;
    private double speed;
    private double turn;
    private long time;
    private long starttime;
    public AutoDriveCommand(DriveSubsystem dS,double speed,double turn,long time){
        this.dS=dS;
        this.speed=speed;
        this.turn=turn;
        this.time=time;
        addRequirements(dS);
    }
    @Override
    public void initialize(){
        starttime=System.currentTimeMillis();
        dS.driveProportional(speed, turn);
    }
    @Override
    public void end(boolean interrupted){
        dS.stop();
    }
    @Override
    public boolean isFinished(){
        return System.currentTimeMillis() >= starttime+time;
    }
}
