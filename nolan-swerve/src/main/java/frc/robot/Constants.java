// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper.GearRatio;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = .60325;
    public static final double WHEEL_DIAMETER = SdsModuleConfigurations.MK4I_L2.getWheelDiameter();
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER*Math.PI;
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
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(162.7); // FIXME Measure and set front
                                                                                        // left steer offset

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 13;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 14;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 20;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(135.08); // FIXME Measure and set front
                                                                                          // right steer offset

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 15;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 16;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 21;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(125); // FIXME Measure and set back left
                                                                                     // steer offset

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 17;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 18;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 22;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(9.45); // FIXME Measure and set back
                                                                                       // right steer offset
    public static DrivetrainSubsystem.Configuration GET_SWERVE_DRIVE_CONFIG() {
        DrivetrainSubsystem.Configuration config = new DrivetrainSubsystem.Configuration();

        config.m_maxVoltage = DRIVE.MAX_VOLTAGE;

        config.m_maxMetersPerSecond = 6380 / 60.0 *
                SdsModuleConfigurations.MK4_L2.getDriveReduction() *
                SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;

        config.m_maxRadiansPerSecond = config.m_maxRadiansPerSecond /
                Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

        return config;
    }

    public static final class DRIVE {
        public static final double GEAR_RATIO = Mk4iSwerveModuleHelper.GearRatio.L2.getConfiguration().getDriveReduction();
        public static final double ENCODER_CLICKS_PER_ROTATION = 2048 / GEAR_RATIO;

        public static final SimpleMotorFeedforward TRAJECTORY_FEEDFORWARD = new SimpleMotorFeedforward(
            Constants.DRIVE.KS_VOLTS,
            Constants.DRIVE.KV_VOLTS_SECONDS_PER_METER,
            Constants.DRIVE.KA_VOLTS_SECONDS_SQUARED_PER_METER);

        public static final double DRIVETRAIN_PX_CONTROLLER = 0.0;
        public static final double DRIVETRAIN_PY_CONTROLLER = 0.0;
        public static final double DRIVETRAIN_PTHETA_CONTROLLER = 0.0;
        public static final double MAX_VOLTAGE = 12.0;

        public static final double MAX_SPEED_METERS_PER_SECOND = 2;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 1;
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 0;
        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 0;
        public static final DifferentialDriveKinematics TANK_DRIVE_KINEMATICS = new DifferentialDriveKinematics(
            DRIVETRAIN_TRACKWIDTH_METERS);;
        
        // public static final SwerveDriveKinematics DRIVE_KINEMATICS = null;
        public static final TrajectoryConstraint TRAJECTORY_VOLTAGE_CONSTRAINT = new DifferentialDriveVoltageConstraint(
            TRAJECTORY_FEEDFORWARD,
            TANK_DRIVE_KINEMATICS,
            MAX_VOLTAGE);

        public static final double RAMSETE_B = 2;
        public static final double RAMSETE_ZETA = 0.7;

        public static final RamseteController TRAJECTORY_RAMSETE_CONTROLLER = new RamseteController(
            Constants.DRIVE.RAMSETE_B, Constants.DRIVE.RAMSETE_ZETA);;
        private static final double KS_VOLTS = 0.6596;
        private static final double KV_VOLTS_SECONDS_PER_METER = 0.045739;
        private static final double KA_VOLTS_SECONDS_SQUARED_PER_METER = 0.0070417;
        
        private static final double P_DRIVE_VEL = 0.56122;
        public static final PIDController TRAJECTORY_DRIVE_PID = new PIDController(Constants.DRIVE.P_DRIVE_VEL, 0, 0);
    }

}
