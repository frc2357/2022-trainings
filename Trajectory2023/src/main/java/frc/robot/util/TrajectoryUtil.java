package frc.robot.util;

import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TrajectoryUtil {
    public static TrajectoryConfig getTrajectoryConfig(boolean isReversed) {
        return new TrajectoryConfig(
                Constants.DRIVE.MAX_SPEED_METERS_PER_SECOND,
                Constants.DRIVE.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(Constants.DRIVE.TANK_DRIVE_KINEMATICS)
                // Apply the voltage constraint
                .addConstraint(Constants.DRIVE.TRAJECTORY_VOLTAGE_CONSTRAINT)
                .setReversed(isReversed);
    }

    public static SequentialCommandGroup createTrajectoryPathCommand(DrivetrainSubsystem driveSub,
            List<Pose2d> waypoints, boolean reversed, boolean resetOdometry) {
        
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints,
                getTrajectoryConfig(reversed));
        return createDrivePathCommand(driveSub, trajectory, resetOdometry);
    }

    public static SequentialCommandGroup createTrajectoryPathCommand(DrivetrainSubsystem driveSub,
            Pose2d start, List<Translation2d> middle, Pose2d end, boolean reversed, boolean resetOdometry) {

        TrajectoryConfig config = getTrajectoryConfig(reversed);
        // System.out.print("config.getConstraints()");
        // System.out.println(config.getConstraints());
        // System.out.print("config.getStartVelocity()");
        // System.out.println(config.getStartVelocity());
        // System.out.print("config.getEndVelocity()");
        // System.out.println(config.getEndVelocity());
        // System.out.print("config.getMaxVelocity()");
        // System.out.println(config.getMaxVelocity());
        // System.out.print("config.getMaxAcceleration()");
        // System.out.println(config.getMaxAcceleration());
        // System.out.print("config.isReversed()");
        // System.out.println(config.isReversed());

        // System.out.print("start: ");
        // System.out.println(start);
        // System.out.print("middle: ");
        // System.out.println(middle);
        // System.out.print("end: ");
        // System.out.println(end);
        // System.out.print("getTrajectoryConfig(reversed): ");
        // System.out.println(getTrajectoryConfig(reversed));

        // System.out.println(Constants.DRIVE.TRAJECTORY_FEEDFORWARD);


        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(start, middle, end,
                getTrajectoryConfig(reversed));
        System.out.println(trajectory);
        return createDrivePathCommand(driveSub, trajectory, resetOdometry);
    }

    public static SequentialCommandGroup createDrivePathCommand(DrivetrainSubsystem driveSub,
            Trajectory trajectory, boolean resetOdometry) {
        SequentialCommandGroup pathCommand = new SequentialCommandGroup();

        if (resetOdometry) {
            pathCommand.addCommands(new InstantCommand(() -> {
                driveSub.resetOdometry(trajectory.getInitialPose());
            }, driveSub));
        }

        pathCommand.addCommands(createRamseteCommand(driveSub, trajectory));

        pathCommand.addCommands(new InstantCommand(() -> {
            driveSub.drive(new ChassisSpeeds(0, 0, 0));
        }));

        return pathCommand;
    }

    public static RamseteCommand createRamseteCommand(DrivetrainSubsystem driveSub, Trajectory trajectory) {
        return new RamseteCommand(trajectory,
            () -> driveSub.getPose(),
            Constants.DRIVE.TRAJECTORY_RAMSETE_CONTROLLER,
            Constants.DRIVE.TRAJECTORY_FEEDFORWARD,
            Constants.DRIVE.TANK_DRIVE_KINEMATICS,
            () -> driveSub.getWheelSpeeds(),
            Constants.DRIVE.TRAJECTORY_DRIVE_PID,
            Constants.DRIVE.TRAJECTORY_DRIVE_PID,
            (leftVolts, rightVolts) -> driveSub.setTankDriveVolts(leftVolts, rightVolts),
            driveSub
        );

        
    }
}
