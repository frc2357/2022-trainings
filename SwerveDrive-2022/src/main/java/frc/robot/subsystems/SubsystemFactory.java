package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import frc.robot.Constants;

public class SubsystemFactory {
    
    public SubsystemFactory() {}

    public SwerveDriveSubsystem CreateSwerveDriveSubsystem() {
        SwerveModule frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
            Mk4SwerveModuleHelper.GearRatio.L2,
            Constants.CAN_ID.FRONT_LEFT_DRIVE,
            Constants.CAN_ID.FRONT_LEFT_ANGLE,
            Constants.CAN_ID.FRONT_LEFT_ENCODER,
            Constants.SWERVE.FRONT_LEFT_OFFSET
        );

        SwerveModule frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
            Mk4SwerveModuleHelper.GearRatio.L2,
            Constants.CAN_ID.FRONT_RIGHT_DRIVE,
            Constants.CAN_ID.FRONT_RIGHT_ANGLE,
            Constants.CAN_ID.FRONT_RIGHT_ENCODER,
            Constants.SWERVE.FRONT_RIGHT_OFFSET
        );

        SwerveModule backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
            Mk4SwerveModuleHelper.GearRatio.L2,
            Constants.CAN_ID.BACK_LEFT_DRIVE,
            Constants.CAN_ID.BACK_LEFT_ANGLE,
            Constants.CAN_ID.BACK_LEFT_ENCODER,
            Constants.SWERVE.BACK_LEFT_OFFSET
        );

        SwerveModule backRightModule = Mk4SwerveModuleHelper.createFalcon500(
            Mk4SwerveModuleHelper.GearRatio.L2,
            Constants.CAN_ID.BACK_RIGHT_DRIVE,
            Constants.CAN_ID.BACK_RIGHT_ANGLE,
            Constants.CAN_ID.BACK_RIGHT_ENCODER,
            Constants.SWERVE.BACK_RIGHT_OFFSET
        );

        PigeonIMU gyro = new PigeonIMU(Constants.CAN_ID.GYRO_ID);

        SwerveDriveSubsystem subsystem = new SwerveDriveSubsystem(frontLeftModule, frontRightModule, backLeftModule, backRightModule, gyro);

        subsystem.configure(Constants.SWERVE.GET_SWERVE_DRIVE_CONFIG());

        return subsystem;
    }

}
