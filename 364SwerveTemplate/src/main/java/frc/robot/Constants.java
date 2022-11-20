// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class CAN_ID {
        public static final int GYRO = 5;
    }

    public static final class SWERVE {
        public static final double WHEEL_BASE = 24.75;
        public static final double TRACK_WIDTH = 23.75;
        public static final double WHEEL_DIAMETER = 0;
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

        public static final double DRIVE_GEAR_RATIO = 0.0;
        public static final double TURN_GEAR_RATIO = 0.0;

        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)
        );

        public static final int TURN_CONTINUOUS_CURRENT_LIMIT = 25;
        public static final int TURN_PEAK_CURRENT_LIMIT = 40;
        public static final double TURN_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean TURN_ENABLE_CURRENT_LIMIT = false;
        
        public static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 35;
        public static final int DRIVE_PEAK_CURRENT_LIMIT = 60;
        public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = false;
        
        public static final double TURN_KP = 0.6;
        public static final double TURN_KI = 0.0;
        public static final double TURN_KD = 12.0;
        public static final double TURN_KF = 0.0;

        public static final double DRIVE_KP = 0.1;
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;
        public static final double DRIVE_KF = 0.0;

        public static final double DRIVE_KS = 0;
        public static final double DRIVE_KV = 0;
        public static final double DRIVE_KA = 0;

        public static final double MAX_SPEED_METER_PER_SECOND = 4.5;
        public static final double MAX_RADIANS_PER_SECOND = 11.5;

        public static final NeutralMode TURN_NEUTRAL_MODE = NeutralMode.Coast;
        public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Brake;

        public static final boolean DRIVE_INVERT = false;
        public static final boolean TURN_INVERT = false;
        public static final boolean CANCODER_INVERT = false;
        public static final boolean INVERT_GYRO = false;

        public static final class FRONT_LEFT_MODULE {
            public static final int DRIVE_MOTOR_ID = 11;
            public static final int TURN_MOTOR_ID = 12;
            public static final int CANCODER_ID = 19;
            public static final double OFFSET = 0.0; //TODO: Update this
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_MOTOR_ID, TURN_MOTOR_ID, CANCODER_ID, OFFSET);
        }

        public static final class FRONT_RIGHT_MODULE {
            public static final int DRIVE_MOTOR_ID = 13;
            public static final int TURN_MOTOR_ID = 14;
            public static final int CANCODER_ID = 20;
            public static final double OFFSET = 0.0; //TODO: Update this
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_MOTOR_ID, TURN_MOTOR_ID, CANCODER_ID, OFFSET);
        }

        public static final class BACK_LEFT_MODULE {
            public static final int DRIVE_MOTOR_ID = 15;
            public static final int TURN_MOTOR_ID = 16;
            public static final int CANCODER_ID = 21;
            public static final double OFFSET = 0.0; //TODO: Update this
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_MOTOR_ID, TURN_MOTOR_ID, CANCODER_ID, OFFSET);
        }

        public static final class BACK_RIGHT_MODULE {
            public static final int DRIVE_MOTOR_ID = 17;
            public static final int TURN_MOTOR_ID = 18;
            public static final int CANCODER_ID = 22;
            public static final double OFFSET = 0.0; //TODO: Update this
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_MOTOR_ID, TURN_MOTOR_ID, CANCODER_ID, OFFSET);
        }


    }

}
