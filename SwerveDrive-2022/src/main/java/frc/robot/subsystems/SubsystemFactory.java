package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;

public class SubsystemFactory {
    
    public SubsystemFactory() {}

    public SwerveDriveSubsystem CreateSwerveDriveSubsystem() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        
        SwerveModule frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(0,0),
            Mk4iSwerveModuleHelper.GearRatio.L2,
            Constants.CAN_ID.FRONT_LEFT_DRIVE,
            Constants.CAN_ID.FRONT_LEFT_ANGLE,
            Constants.CAN_ID.FRONT_LEFT_ENCODER,
            Constants.SWERVE.FRONT_LEFT_OFFSET
        );

        SwerveModule frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(2,0),
            Mk4iSwerveModuleHelper.GearRatio.L2,
            Constants.CAN_ID.FRONT_RIGHT_DRIVE,
            Constants.CAN_ID.FRONT_RIGHT_ANGLE,
            Constants.CAN_ID.FRONT_RIGHT_ENCODER,
            Constants.SWERVE.FRONT_RIGHT_OFFSET
        );

        SwerveModule backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(4,0),
            Mk4iSwerveModuleHelper.GearRatio.L2,
            Constants.CAN_ID.BACK_LEFT_DRIVE,
            Constants.CAN_ID.BACK_LEFT_ANGLE,
            Constants.CAN_ID.BACK_LEFT_ENCODER,
            Constants.SWERVE.BACK_LEFT_OFFSET
        );

        SwerveModule backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(6,0),
            Mk4iSwerveModuleHelper.GearRatio.L2,
            Constants.CAN_ID.BACK_RIGHT_DRIVE,
            Constants.CAN_ID.BACK_RIGHT_ANGLE,
            Constants.CAN_ID.BACK_RIGHT_ENCODER,
            Constants.SWERVE.BACK_RIGHT_OFFSET
        );

        WPI_Pigeon2 gyro = new WPI_Pigeon2(Constants.CAN_ID.GYRO_ID);

        SwerveDriveSubsystem subsystem = new SwerveDriveSubsystem(frontLeftModule, frontRightModule, backLeftModule, backRightModule, gyro);

        subsystem.configure(Constants.SWERVE.GET_SWERVE_DRIVE_CONFIG());

        return subsystem;
    }

}
