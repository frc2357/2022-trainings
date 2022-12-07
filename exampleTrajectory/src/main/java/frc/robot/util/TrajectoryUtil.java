package com.team2357.frc2022.util;

import java.util.List;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class TrajectoryUtil {
    public static TrajectoryConfig getTrajectoryConfig(boolean isReversed) {
        return new TrajectoryConfig(
                Constants.DRIVE.MAX_SPEED_METERS_PER_SECOND,
                Constants.DRIVE.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.DRIVE.DRIVE_KINEMATICS)
                        // Apply the voltage constraint
                        .addConstraint(Constants.DRIVE.TRAJECTORY_VOLTAGE_CONSTRAINT)
                        .setReversed(isReversed);
    }

    public static SequentialCommandGroup createTrajectoryPathCommand(DriveSubsystem driveSub,
            List<Pose2d> waypoints, boolean reversed, boolean resetOdometry) {

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints,
                getTrajectoryConfig(reversed));
        return createDrivePathCommand(driveSub, trajectory, resetOdometry);
    }

    public static SequentialCommandGroup createTrajectoryPathCommand(DriveSubsystem driveSub,
            Pose2d start, List<Translation2d> middle, Pose2d end, boolean reversed, boolean resetOdometry) {

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(start, middle, end,
                getTrajectoryConfig(reversed));
        return createDrivePathCommand(driveSub, trajectory, resetOdometry);
    }

    public static SequentialCommandGroup createDrivePathCommand(DriveSubsystem driveSub,
            Trajectory trajectory, boolean resetOdometry) {
        SequentialCommandGroup pathCommand = new SequentialCommandGroup();

        if (resetOdometry) {
            pathCommand.addCommands(new InstantCommand(() -> {
                driveSub.resetOdometry(trajectory.getInitialPose());
            }, driveSub));
        }

        pathCommand.addCommands(createRamseteCommand(driveSub, trajectory));

        pathCommand.addCommands(new InstantCommand(() -> {
            driveSub.drive(0, 0, 0, true);
        }));

        return pathCommand;
    }

    public static RamseteCommand createRamseteCommand(DriveSubsystem driveSub, Trajectory trajectory) {
        return new RamseteCommand(trajectory,
                driveSub::getPose,
                Constants.DRIVE.TRAJECTORY_RAMSETE_CONTROLLER,
                Constants.DRIVE.TRAJECTORY_FEEDFORWARD,
                Constants.DRIVE.DRIVE_KINEMATICS,
                driveSub::getWheelSpeeds,
                Constants.DRIVE.TRAJECTORY_DRIVE_PID,
                Constants.DRIVE.TRAJECTORY_DRIVE_PID,
                // RamseteCommand passes volts to the callback
                driveSub::setTankDriveVolts,
                driveSub);
    }
}
