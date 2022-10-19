package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveSubsystem extends SubsystemBase {
    private static SwerveDriveSubsystem instance = null;

    public static SwerveDriveSubsystem getInstance() {
        return instance;
    }

    public static class Configuration {

    }

    private PigeonIMU m_gyro;
    private boolean m_isGyroReversed;

    private Configuration m_config;

}
