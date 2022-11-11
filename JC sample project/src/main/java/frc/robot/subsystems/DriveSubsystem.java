package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase{
    private WPI_TalonFX m_leftfalconMaster;
    private WPI_TalonFX m_rightfalconMaster;

    private WPI_TalonFX[] m_leftfalconFollowers;
    private WPI_TalonFX[] m_rightfalconFollowers;
    public DriveSubsystem(WPI_TalonFX leftMaster,WPI_TalonFX[] leftFollowers,WPI_TalonFX rightMaster,WPI_TalonFX[] rightFollowers,boolean isRightInverted){
        m_leftfalconFollowers = leftFollowers;
        m_leftfalconMaster = leftMaster;
        m_rightfalconFollowers = rightFollowers;
        m_rightfalconMaster = rightMaster;
        m_leftfalconMaster.configFactoryDefault();
        m_rightfalconMaster.configFactoryDefault();
        m_rightfalconMaster.setInverted(isRightInverted);
        m_leftfalconMaster.setInverted(!isRightInverted);
        for(WPI_TalonFX fl:m_leftfalconFollowers){     
            fl.configFactoryDefault();
            fl.setInverted(!isRightInverted);
            fl.setNeutralMode(NeutralMode.Brake);
            fl.follow(m_leftfalconMaster);
        }
        for(WPI_TalonFX fl:m_rightfalconFollowers){     
            fl.configFactoryDefault();
            fl.setInverted(isRightInverted);
            fl.setNeutralMode(NeutralMode.Brake);
            fl.follow(m_rightfalconMaster);
        }
    }
    public void stop(){
        m_leftfalconMaster.set(0);
        m_rightfalconMaster.set(0);
    }
    public void driveProportional(double speed,double turn){
        m_leftfalconMaster.set(MathUtil.clamp(speed-turn, -1, 1));
        m_rightfalconMaster.set(MathUtil.clamp(speed+turn, -1, 1));
    }
}
