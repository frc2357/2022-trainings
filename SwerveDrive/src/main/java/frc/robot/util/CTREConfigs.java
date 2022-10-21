package frc.robot.util;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import frc.robot.Constants;

public final class CTREConfigs {
    public static TalonFXConfiguration swerveDriveFXConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.SWERVE_CONSTANTS.DRIVE_ENABLE_CURRENT_LIMIT, 
            Constants.SWERVE_CONSTANTS.DRIVE_CONTINUOUS_CURRENT_LIMIT, 
            Constants.SWERVE_CONSTANTS.DRIVE_PEAK_CURRENT_LIMIT, 
            Constants.SWERVE_CONSTANTS.DRIVE_PEAK_CURRENT_DURATION);

        config.slot0.kP = Constants.SWERVE_CONSTANTS.DRIVE_KP;
        config.slot0.kI = Constants.SWERVE_CONSTANTS.DRIVE_KI;
        config.slot0.kD = Constants.SWERVE_CONSTANTS.DRIVE_KD;
        config.slot0.kF = Constants.SWERVE_CONSTANTS.DRIVE_KF;        
        config.supplyCurrLimit = driveSupplyLimit;
        config.initializationStrategy = SensorInitializationStrategy.BootToZero;
        config.openloopRamp = Constants.SWERVE_CONSTANTS.OPEN_LOOP_RAMP;
        config.closedloopRamp = Constants.SWERVE_CONSTANTS.CLOSED_LOOP_RAMP;
        return config;
    }

    public static TalonFXConfiguration swerveAngleFXConfig() {
        TalonFXConfiguration angleConfig = new TalonFXConfiguration();
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.SWERVE_CONSTANTS.ANGLE_ENABLE_CURRENT_LIMIT, 
            Constants.SWERVE_CONSTANTS.ANGLE_CONTINUOUS_CURRENT_LIMIT, 
            Constants.SWERVE_CONSTANTS.ANGLE_PEAK_CURRENT_LIMIT, 
            Constants.SWERVE_CONSTANTS.ANGLE_PEAK_CURRENT_DURATION);

        angleConfig.slot0.kP = Constants.SWERVE_CONSTANTS.ANGLE_KP;
        angleConfig.slot0.kI = Constants.SWERVE_CONSTANTS.ANGLE_KI;
        angleConfig.slot0.kD = Constants.SWERVE_CONSTANTS.ANGLE_KD;
        angleConfig.slot0.kF = Constants.SWERVE_CONSTANTS.ANGLE_KF;
        angleConfig.supplyCurrLimit = angleSupplyLimit;
        angleConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        return angleConfig;
    }

    public static CANCoderConfiguration swerveCancoderConfig() {
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.sensorDirection = Constants.SWERVE_CONSTANTS.CANCODER_INVERT;
        config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        config.sensorTimeBase = SensorTimeBase.PerSecond;
        return config;
    }
}