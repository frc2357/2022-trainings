// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.SwerveModule;
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

        public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 11;
        public static final int FRONT_LEFT_TURN_MOTOR_ID = 12;
        public static final int FRONT_LEFT_CANCODER_ID = 19;

        public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 13;
        public static final int FRONT_RIGHT_TURN_MOTOR_ID = 14;
        public static final int FRONT_RIGHT_CANCODER_ID = 20;

        public static final int BACK_LEFT_DRIVE_MOTOR_ID = 15;
        public static final int BACK_LEFT_TURN_MOTOR_ID = 16;
        public static final int BACK_LEFT_CANCODER_ID = 21;

        public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 17;
        public static final int BACK_RIGHT_TURN_MOTOR_ID = 18;
        public static final int BACK_RIGHT_CANCODER_ID = 22;
        
    }

    public static final class SWERVE {

        public static final class MODULE_OFFSETS {
            public static final double FRONT_LEFT = 0.0;
            public static final double FRONT_RIGHT = 0.0;
            public static final double BACK_LEFT = 0.0;
            public static final double BACK_RIGHT = 0.0;
        }

        public static final double MAX_SPEED_METER_PER_SECOND = 4.5;
        public static final double MAX_RADIANS_PER_SECOND = 11.5;

        public static final SwerveDriveSubsystem.Configuration GET_SWERVE_SUBSYSTEM_CONFIGURATION() {
            SwerveDriveSubsystem.Configuration config = new SwerveDriveSubsystem.Configuration();

            config.m_wheelBase = 24.75;
            config.m_trackWidth = 23.75;
            config.m_wheelDiameter = 0.0;
            config.m_wheelCircumference = config.m_wheelDiameter * Math.PI;

            config.m_driveGearRatio = 0.0;
            config.m_turnGearRatio = 0.0;

            config.m_kinematics = new SwerveDriveKinematics(
                new Translation2d(config.m_wheelBase / 2.0, config.m_trackWidth / 2.0),
                new Translation2d(config.m_wheelBase / 2.0, -config.m_trackWidth / 2.0),
                new Translation2d(-config.m_wheelBase / 2.0, config.m_trackWidth / 2.0),
                new Translation2d(-config.m_wheelBase / 2.0, -config.m_trackWidth / 2.0)
            );

            config.m_turnContinuousCurrentLimit = 25;
            config.m_turnPeakCurrentLimit = 40;
            config.m_turnPeakCurrentDuration = 0.1;
            config.m_turnEnableCurrentLimit = false;
            config.m_turnCurrentLimitConfig = new SupplyCurrentLimitConfiguration(
                config.m_turnEnableCurrentLimit,
                config.m_turnContinuousCurrentLimit,
                config.m_turnPeakCurrentLimit,
                config.m_turnPeakCurrentDuration
            );

            config.m_driveContinuousCurrentLimit = 35;
            config.m_drivePeakCurrentLimit = 60;
            config.m_drivePeakCurrentDuration = 0.1;
            config.m_driveEnableCurrentLimit = false;
            config.m_driveCurrentLimitConfig = new SupplyCurrentLimitConfiguration(
                config.m_driveEnableCurrentLimit,
                config.m_driveContinuousCurrentLimit,
                config.m_drivePeakCurrentLimit,
                config.m_drivePeakCurrentDuration
            );

            config.m_turnKP = 0.6;
            config.m_turnKI = 0.0;
            config.m_turnKD = 12.0;
            config.m_turnKF = 0.0;

            config.m_driveKP = 0.1;
            config.m_driveKI = 0.0;
            config.m_driveKD = 0.0;
            config.m_driveKF = 0.0;

            config.m_driveKS = 0.0;
            config.m_driveKV = 0.0;
            config.m_driveKA = 0.0;

            config.m_turnNeutralMode = NeutralMode.Coast;
            config.m_driveNeutralMode = NeutralMode.Brake;

            config.m_driveInvert = false;
            config.m_turnInvert = false;
            config.m_cancoderInvert = false;
            config.m_gyroInvert = false;

            config.m_frontLeftModuleConstants = new SwerveModuleConstants(
                CAN_ID.FRONT_LEFT_DRIVE_MOTOR_ID,   // frontLeftDriveMotorID
                CAN_ID.FRONT_LEFT_TURN_MOTOR_ID,    // frontLeftTurnMotorID
                CAN_ID.FRONT_LEFT_CANCODER_ID,      // frontLeftCANCoderID
                SWERVE.MODULE_OFFSETS.FRONT_LEFT    // frontLeftAngleOffset
            );            
            config.m_frontRightModuleConstants = new SwerveModuleConstants(
                CAN_ID.FRONT_RIGHT_DRIVE_MOTOR_ID,   // frontRightDriveMotorID
                CAN_ID.FRONT_RIGHT_TURN_MOTOR_ID,    // frontRightTurnMotorID
                CAN_ID.FRONT_RIGHT_CANCODER_ID,      // frontRightCANCoderID
                SWERVE.MODULE_OFFSETS.FRONT_RIGHT    // frontRightAngleOffset
            );
            config.m_backLeftModuleConstants = new SwerveModuleConstants(
                CAN_ID.BACK_LEFT_DRIVE_MOTOR_ID,   // backLeftDriveMotorID
                CAN_ID.BACK_LEFT_TURN_MOTOR_ID,    // backLeftTurnMotorID
                CAN_ID.BACK_LEFT_CANCODER_ID,      // backLeftCANCoderID
                SWERVE.MODULE_OFFSETS.BACK_LEFT    // backLeftAngleOffset
            );
            config.m_backRightModuleConstants = new SwerveModuleConstants(
                CAN_ID.BACK_RIGHT_DRIVE_MOTOR_ID,   // backRightDriveMotorID
                CAN_ID.BACK_RIGHT_TURN_MOTOR_ID,    // backRightTurnMotorID
                CAN_ID.BACK_RIGHT_CANCODER_ID,      // backRightCANCoderID
                SWERVE.MODULE_OFFSETS.BACK_RIGHT    // backRightAngleOffset
            );

            config.m_turnMotorConfig = new TalonFXConfiguration();
            config.m_driveMotorConfig = new TalonFXConfiguration();
            config.m_cancoderConfig = new CANCoderConfiguration();

            config.m_turnMotorConfig.slot0.kP = config.m_turnKP;
            config.m_turnMotorConfig.slot0.kI = config.m_turnKI;
            config.m_turnMotorConfig.slot0.kD = config.m_turnKD;
            config.m_turnMotorConfig.slot0.kF = config.m_turnKF;
            config.m_turnMotorConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;


            config.m_driveMotorConfig.slot0.kP = config.m_driveKP;
            config.m_driveMotorConfig.slot0.kI = config.m_driveKI;
            config.m_driveMotorConfig.slot0.kD = config.m_driveKD;
            config.m_driveMotorConfig.slot0.kF = config.m_driveKF;
            config.m_driveMotorConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;

            config.m_cancoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
            config.m_cancoderConfig.sensorDirection = config.m_cancoderInvert;
            config.m_cancoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
            config.m_cancoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;

            return config;
        }
        
    }
}
