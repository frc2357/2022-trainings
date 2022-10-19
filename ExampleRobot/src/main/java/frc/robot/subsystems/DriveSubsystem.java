package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase{

    private WPI_TalonFX m_leftFalconMaster;
    private WPI_TalonFX m_rightFalconMaster;

    private WPI_TalonFX[] m_leftFalconFollowers;
    private WPI_TalonFX[] m_rightFalconFollowers;


    public DriveSubsystem(WPI_TalonFX leftFalconMaster, WPI_TalonFX[] leftFalconFollowers, WPI_TalonFX rightFalconMaster, WPI_TalonFX[] rightFalconFollowers, boolean isRightInverted) {
        m_leftFalconMaster = leftFalconMaster;
        m_rightFalconMaster = rightFalconMaster;

        m_leftFalconFollowers = leftFalconFollowers;
        m_rightFalconFollowers = rightFalconFollowers;

        m_leftFalconMaster.configFactoryDefault();
        m_leftFalconMaster.setInverted(!isRightInverted);

        m_rightFalconMaster.configFactoryDefault();
        m_rightFalconMaster.setInverted(isRightInverted);

        for (WPI_TalonFX follower : m_leftFalconFollowers) {
            follower.configFactoryDefault();
            follower.setInverted(!isRightInverted);
            follower.setNeutralMode(NeutralMode.Brake);
            follower.follow(m_leftFalconMaster, FollowerType.PercentOutput);
        }

        for(WPI_TalonFX follower : m_rightFalconFollowers) {
            follower.configFactoryDefault();
            follower.setInverted(isRightInverted);
            follower.setNeutralMode(NeutralMode.Brake);
            follower.follow(m_rightFalconMaster, FollowerType.PercentOutput);
        }
    }

    /**
     * 
     * @param speed -1 to 1
     * @param turn -1 to 1
     */
    public void driveProportional(double speed, double turn) {
        double leftProportion = speed - turn;
        double rightProportion = speed + turn;

        leftProportion = Math.min(leftProportion, 1);
        leftProportion = Math.max(leftProportion, -1);

        rightProportion = Math.min(rightProportion, 1);
        rightProportion = Math.max(rightProportion, -1);

        m_leftFalconMaster.set(leftProportion);
        m_rightFalconMaster.set(rightProportion);
    }
}
