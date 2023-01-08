package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;

public class SubsystemFactory {
    public SwerveDriveSubsystem CreateSwerveDriveSubsystem() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        WPI_Pigeon2 pigeon = new WPI_Pigeon2(Constants.DRIVETRAIN_PIGEON_ID);

        SwerveModule frontLeft = Mk4iSwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
                Mk4iSwerveModuleHelper.GearRatio.L2,
                Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
                Constants.FRONT_LEFT_MODULE_STEER_MOTOR,
                Constants.FRONT_LEFT_MODULE_STEER_ENCODER,
                // the canbus the module is on
                Constants.DRIVE_CANBUS,
                Constants.FRONT_LEFT_MODULE_STEER_OFFSET);

        SwerveModule frontRight = Mk4iSwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(2, 0),
                Mk4iSwerveModuleHelper.GearRatio.L2,
                Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                Constants.FRONT_RIGHT_MODULE_STEER_MOTOR,
                Constants.FRONT_RIGHT_MODULE_STEER_ENCODER,
                // the canbus the module is on
                Constants.DRIVE_CANBUS,
                Constants.FRONT_RIGHT_MODULE_STEER_OFFSET);

        SwerveModule backLeft = Mk4iSwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(4, 0),
                Mk4iSwerveModuleHelper.GearRatio.L2,
                Constants.BACK_LEFT_MODULE_DRIVE_MOTOR,
                Constants.BACK_LEFT_MODULE_STEER_MOTOR,
                Constants.BACK_LEFT_MODULE_STEER_ENCODER,
                // the canbus the module is on
                Constants.DRIVE_CANBUS,
                Constants.BACK_LEFT_MODULE_STEER_OFFSET);

        SwerveModule backRight = Mk4iSwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(6, 0),
                Mk4iSwerveModuleHelper.GearRatio.L2,
                Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
                Constants.BACK_RIGHT_MODULE_STEER_MOTOR,
                Constants.BACK_RIGHT_MODULE_STEER_ENCODER,
                // the canbus the module is on
                Constants.DRIVE_CANBUS,
                Constants.BACK_RIGHT_MODULE_STEER_OFFSET);

        SwerveDriveSubsystem subsystem = new SwerveDriveSubsystem(pigeon, frontLeft, frontRight, backLeft, backRight);
        subsystem.configure(Constants.GET_SWERVE_DRIVE_CONFIG());

        return subsystem;
    }
}
