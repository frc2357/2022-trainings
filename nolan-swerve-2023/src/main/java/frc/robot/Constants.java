// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    /**
     * The canbus the swerve modules are on
     * 
     * Use "" or "rio" for rio can bus
     * 
     * Use name of Canivore device to use Canivore
     *  */ 
    public static final String DRIVE_CANBUS = "rio";

    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = .60325;
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = .62865; 

    public static final int DRIVETRAIN_PIGEON_ID = 5;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 11; 
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 12; 
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 19; 
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(162.7); // Init 162.7

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 13; 
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 14; 
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 20; 
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(135.08);//-Math.toRadians(135.08); // FIXME Measure and set front right steer offset

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 15; 
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 16; 
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 21; 
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(125); // FIXME Measure and set back left steer offset

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 17; 
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 18; 
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 22; 
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(9.45); // FIXME Measure and set back right steer offset

	public static final double DEADBAND = 0.1;

    public static SwerveDriveSubsystem.Configuration GET_SWERVE_DRIVE_CONFIG() {
        SwerveDriveSubsystem.Configuration config = new SwerveDriveSubsystem.Configuration();

        config.m_trackwidthMeters = .60325;
        config.m_wheelbaseMeters = .62865;

        config.m_maxVoltage = 10.0;
        config.m_maxVelocityMetersPerSecond = 6380.0 / 60.0 *
                SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
                SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;
        config.m_maxAngularVelocityRadiansPerSecond = config.m_maxVelocityMetersPerSecond /
                Math.hypot(config.m_trackwidthMeters / 2.0, config.m_wheelbaseMeters / 2.0);

        config.m_xController = new PIDController(.56122, 0, 0);
        config.m_yController = new PIDController(.56122, 0, 0);
        config.m_thetaController = new PIDController(2.15, 0, 0);
        
        return config;
    }
}
