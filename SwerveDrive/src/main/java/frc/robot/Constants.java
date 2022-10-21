package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.util.SwerveModuleConstants;

public final class Constants {

    public static final class SWERVE_CONSTANTS {
        // Drivetrain constants
        public static final double TRACK_WIDTH = 0;
        public static final double WHEEL_BASE = 0;
        public static final double WHEEL_DIAMETER = 0;
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

        public static final double MAX_SPEED = 0;
        public static final double DRIVE_GEAR_RATIO = 1.0;
        public static final double ANGLE_GEAR_RATIO = 1.0;

        public static final Translation2d[] SWERVE_MODULE_OFFSETS = {
            new Translation2d(WHEEL_BASE/2.0, TRACK_WIDTH/2.0),
            new Translation2d(WHEEL_BASE/2.0, -TRACK_WIDTH/2.0),
            new Translation2d(-WHEEL_BASE/2.0, TRACK_WIDTH/2.0),
            new Translation2d(-WHEEL_BASE/2.0, -TRACK_WIDTH/2.0)
        };

        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(SWERVE_MODULE_OFFSETS);

        // Current limiting
        public static final int ANGLE_CONTINUOUS_CURRENT_LIMIT = 0;
        public static final int ANGLE_PEAK_CURRENT_LIMIT = 0;
        public static final double ANGLE_PEAK_CURRENT_DURATION = 0;
        public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = false;

        public static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 0;
        public static final int DRIVE_PEAK_CURRENT_LIMIT = 0;
        public static final double DRIVE_PEAK_CURRENT_DURATION = 0;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = false;

        // Drive PID
        public static final double DRIVE_KP = 0;
        public static final double DRIVE_KI = 0;
        public static final double DRIVE_KD = 0;
        public static final double DRIVE_KF = 0;

        // Angle PID
        public static final double ANGLE_KP = 0;
        public static final double ANGLE_KI = 0;
        public static final double ANGLE_KD = 0;
        public static final double ANGLE_KF = 0;

        // Module 0 constants (Front left)
        public static final SwerveModuleConstants MOD0 = new SwerveModuleConstants(0, 0, 0, 0);
        // Module 1 constants (Front right)
        public static final SwerveModuleConstants MOD1 = new SwerveModuleConstants(0, 0, 0, 0);
        // Module 2 constants (Back left)
        public static final SwerveModuleConstants MOD2 = new SwerveModuleConstants(0, 0, 0, 0);
        // Module 3 constants (Back right)
        public static final SwerveModuleConstants MOD3 = new SwerveModuleConstants(0, 0, 0, 0);

        public static final double DRIVE_KS = 0;
        public static final double DRIVE_KV = 0;
        public static final double DRIVE_KA = 0;

        public static final boolean CANCODER_INVERT = false;

        public static final double CLOSED_LOOP_RAMP = 0;
        public static final double OPEN_LOOP_RAMP = 0;
        
        public static final int TIMEOUT_MILLIS = 0;
        
        
    }

}
