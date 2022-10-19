// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class SwerveConstants {
        public static final boolean driveEnableCurrentLimit = true;
        public static final double driveContinuousCurrentLimit;
        public static final double drivePeakCurrentLimit;
        public static final double drivePeakCurrentDuration;
        public static final double driveKP;
        public static final double driveKI;
        public static final double driveKD;
        public static final double driveKF;
        public static final double openLoopRamp;
        public static final double closedLoopRamp;
        public static final boolean angleEnableCurrentLimit;
        public static final double angleContinuousCurrentLimit;
        public static final double anglePeakCurrentLimit;
        public static final double anglePeakCurrentDuration;
        public static final double angleKP;
        public static final double angleKI;
        public static final double angleKD;
        public static final double angleKF;
        public static final boolean canCoderInvert;

    }

}
