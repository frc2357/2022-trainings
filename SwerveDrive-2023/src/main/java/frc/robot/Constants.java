package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import frc.robot.subsystems.SwerveDriveSubsystem;

public final class Constants {

    public static final class CAN_ID {

        public static final int FRONT_LEFT_DRIVE = 0;
        public static final int FRONT_LEFT_ANGLE = 0;
        public static final int FRONT_LEFT_ENCODER = 0;

        public static final int FRONT_RIGHT_DRIVE = 0;
        public static final int FRONT_RIGHT_ANGLE = 0;
        public static final int FRONT_RIGHT_ENCODER = 0;

        public static final int BACK_LEFT_DRIVE = 0;
        public static final int BACK_LEFT_ANGLE = 0;
        public static final int BACK_LEFT_ENCODER = 0;

        public static final int BACK_RIGHT_DRIVE = 0;
        public static final int BACK_RIGHT_ANGLE = 0;
        public static final int BACK_RIGHT_ENCODER = 0;
        public static final TalonSRX GYRO_ID = null;

    }

    public static final class CONTROLLER {

        public static final int DRIVE_CONTROLLER_PORT = 0;
        public static final double DRIVE_CONTROLLER_DEADBAND = 0.0;

    }

    public static final class SWERVE {
        // Drivetrain constants
        public static final double TRACK_WIDTH = 0;
        public static final double WHEEL_BASE = 0;
        
        public static final double FRONT_LEFT_OFFSET = 0;
        public static final double FRONT_RIGHT_OFFSET = 0;
        public static final double BACK_LEFT_OFFSET = 0;
        public static final double BACK_RIGHT_OFFSET = 0;
        public static SwerveDriveSubsystem.Configuration GET_SWERVE_DRIVE_CONFIG() {
            SwerveDriveSubsystem.Configuration config = new SwerveDriveSubsystem.Configuration();

            config.m_maxVoltage = 12.0;

            config.m_maxMetersPerSecond = 6380 / 60.0 * 
                SdsModuleConfigurations.MK4_L2.getDriveReduction() * 
                SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;
                
            config.m_maxRadiansPerSecond = config.m_maxRadiansPerSecond /
                Math.hypot(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0);

            return config;
        }
    }

}
