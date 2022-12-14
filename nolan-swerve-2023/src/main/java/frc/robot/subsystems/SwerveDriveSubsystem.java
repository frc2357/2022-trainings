// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveSubsystem extends SubsystemBase {
	private static SwerveDriveSubsystem instance = null;

	public static SwerveDriveSubsystem getInstance() {
		return instance;
	}

	private SwerveDriveKinematics m_kinematics;

	private WPI_Pigeon2 m_pigeon;

	private SwerveModule m_frontLeftModule;
	private SwerveModule m_frontRightModule;
	private SwerveModule m_backLeftModule;
	private SwerveModule m_backRightModule;

	private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

	private Configuration m_config;

	private SwerveDriveOdometry m_odometry;

	public static class Configuration {
		/**
		 * The left-to-right distance between the drivetrain wheels (measured from center to center)
		 */
		public double m_trackwidthMeters;

		/**
		 * The front-to-back distance between the drivetrain wheels (measured from center to center)
		 */
		public double m_wheelbaseMeters;

		/**
		 * The maximum voltage that will be delivered to the drive motors
		 */
		public double m_maxVoltage;

		/**
		 * The maximum velocity of the robot in meters per second
		 * (How fast the robot can drive in a straight line)
		 * 
		 * Formula: 6380 / 60 * <gear ratio> * <wheel diameter> * Math.PI
		 */
		public double m_maxVelocityMetersPerSecond;

		/**
		 * The maximum angular velocity of the robot in radians per second 
		 * (how fast the robot can rotate in place)
		 * 
		 * Formula: m_maxVelocityMetersPerSecond / Math.hypot(m_trackwidthMeters / 2, m_wheelbaseMeters / 2)
		 */
		//
		public double m_maxAngularVelocityRadiansPerSecond;


		public PIDController m_xController;
		public PIDController m_yController;
		public PIDController m_thetaController;
	}

	public SwerveDriveSubsystem(WPI_Pigeon2 pigeon, SwerveModule frontLeft, SwerveModule frontRight,
			SwerveModule backLeft, SwerveModule backRight) {
		m_pigeon = pigeon;

		m_frontLeftModule = frontLeft;
		m_frontRightModule = frontRight;
		m_backLeftModule = backLeft;
		m_backRightModule = backRight;

		instance = this;
	}

	public void configure(Configuration config) {
		m_config = config;

		m_kinematics = new SwerveDriveKinematics(
				new Translation2d(m_config.m_trackwidthMeters / 2.0,
						m_config.m_wheelbaseMeters / 2.0),
				new Translation2d(m_config.m_trackwidthMeters / 2.0,
						-m_config.m_wheelbaseMeters / 2.0),
				new Translation2d(-m_config.m_trackwidthMeters / 2.0,
						m_config.m_wheelbaseMeters / 2.0),
				new Translation2d(-m_config.m_trackwidthMeters / 2.0,
						-m_config.m_wheelbaseMeters / 2.0));

		m_odometry = new SwerveDriveOdometry(m_kinematics, getGyroscopeRotation(),
				new SwerveModulePosition[] { m_frontLeftModule.getPosition(),
						m_frontRightModule.getPosition(),
						m_backLeftModule.getPosition(), m_backRightModule.getPosition() });
	}

	public void zeroGyroscope() {
		m_pigeon.reset();
	}

	public Rotation2d getGyroscopeRotation() {
		return Rotation2d.fromDegrees(m_pigeon.getYaw());
	}

	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	public void resetOdometry(Pose2d pose) {
		m_odometry.resetPosition(
				getGyroscopeRotation(),
				new SwerveModulePosition[] { m_frontLeftModule.getPosition(),
						m_frontRightModule.getPosition(),
						m_backLeftModule.getPosition(), m_backRightModule.getPosition() },
				pose);
	}

	public void drive(double x, double y, double rotation) {
		ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
				x * m_config.m_maxVelocityMetersPerSecond,
				y * m_config.m_maxVelocityMetersPerSecond,
				rotation * m_config.m_maxAngularVelocityRadiansPerSecond,
				getGyroscopeRotation());

		drive(chassisSpeeds);
	}

	public void drive(ChassisSpeeds chassisSpeeds) {
		m_chassisSpeeds = chassisSpeeds;

		m_odometry.update(getGyroscopeRotation(),
				new SwerveModulePosition[] { m_frontLeftModule.getPosition(),
						m_frontRightModule.getPosition(),
						m_backLeftModule.getPosition(), m_backRightModule.getPosition() });

		SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(states, m_config.m_maxVelocityMetersPerSecond);

		m_frontLeftModule.set(
				states[0].speedMetersPerSecond / m_config.m_maxVelocityMetersPerSecond * m_config.m_maxVoltage,
				states[0].angle.getRadians());
		m_frontRightModule.set(
				states[1].speedMetersPerSecond / m_config.m_maxVelocityMetersPerSecond * m_config.m_maxVoltage,
				states[1].angle.getRadians());
		m_backLeftModule.set(
				states[2].speedMetersPerSecond / m_config.m_maxVelocityMetersPerSecond * m_config.m_maxVoltage,
				states[2].angle.getRadians());
		m_backRightModule.set(
				states[3].speedMetersPerSecond / m_config.m_maxVelocityMetersPerSecond * m_config.m_maxVoltage,
				states[3].angle.getRadians());
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Angle", m_pigeon.getYaw());

		SmartDashboard.putNumber("Yaw", m_pigeon.getYaw());
		SmartDashboard.putNumber("Pose X", m_odometry.getPoseMeters().getX());
		SmartDashboard.putNumber("Pose Y", m_odometry.getPoseMeters().getY());
		SmartDashboard.putNumber("Pose Angle", m_odometry.getPoseMeters().getRotation().getDegrees());
	}

	// TODO Abstract this function out similair to 2022 code
	// Pick back up here with path following constant placeholders
	public SequentialCommandGroup followPathCommand(final boolean shouldResetOdometry, String trajectoryFileName) {

		// final Trajectory trajectory = generateTrajectory(waypoints);
		final PathPlannerTrajectory trajectory = PathPlanner.loadPath(trajectoryFileName, 2, 3);
		// double Seconds = 0.0;
		// System.out.println("===== Begin Sampling path =====");
		// while(trajectory.getTotalTimeSeconds() > Seconds) {
		// PathPlannerState state = (PathPlannerState) trajectory.sample(Seconds);
		// System.out.println(
		// "time: " + Seconds
		// + ", x: " + state.poseMeters.getX()
		// + ", y: " + state.poseMeters.getY()
		// + ", angle: " + state.poseMeters.getRotation().getDegrees()
		// + ", holo: " + state.holonomicRotation.getDegrees()
		// );
		// Seconds += 0.25;
		// }
		// System.out.println("===== End Sampling Path =====");
		return new InstantCommand(() -> {
			if (shouldResetOdometry) {
				PathPlannerState initialSample = (PathPlannerState) trajectory.sample(0);
				Pose2d initialPose = new Pose2d(initialSample.poseMeters.getTranslation(),
						initialSample.holonomicRotation);
				resetOdometry(initialPose);
			}
			m_config.m_xController.reset();
			m_config.m_yController.reset();
		}).andThen(new PPSwerveControllerCommand(
				trajectory,
				() -> getPose(),
				m_kinematics,
				m_config.m_xController,
				m_config.m_yController,
				m_config.m_thetaController,
				(SwerveModuleState[] moduleStates) -> {
					drive(m_kinematics.toChassisSpeeds(moduleStates));
				},
				this)).andThen(() -> drive(new ChassisSpeeds()), this);
	}

}
