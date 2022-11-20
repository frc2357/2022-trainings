package frc.robot.util;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorTimeBase;

import frc.robot.Constants;

public class CTREConfigs {
    
    public TalonFXConfiguration swerveTurnFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCANCoderConfig;

    public CTREConfigs() {
        swerveTurnFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCANCoderConfig = new CANCoderConfiguration();

        SupplyCurrentLimitConfiguration turnSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.SWERVE.TURN_ENABLE_CURRENT_LIMIT,
            Constants.SWERVE.TURN_CONTINUOUS_CURRENT_LIMIT,
            Constants.SWERVE.TURN_PEAK_CURRENT_LIMIT,
            Constants.SWERVE.TURN_PEAK_CURRENT_DURATION
        );

        swerveTurnFXConfig.slot0.kP = Constants.SWERVE.TURN_KP;
        swerveTurnFXConfig.slot0.kI = Constants.SWERVE.TURN_KI;
        swerveTurnFXConfig.slot0.kD = Constants.SWERVE.TURN_KD;
        swerveTurnFXConfig.slot0.kF = Constants.SWERVE.TURN_KF;
        swerveTurnFXConfig.supplyCurrLimit = turnSupplyLimit;
        swerveTurnFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;

        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.SWERVE.DRIVE_ENABLE_CURRENT_LIMIT,
            Constants.SWERVE.DRIVE_CONTINUOUS_CURRENT_LIMIT,
            Constants.SWERVE.DRIVE_PEAK_CURRENT_LIMIT,
            Constants.SWERVE.DRIVE_PEAK_CURRENT_DURATION
        );

        swerveDriveFXConfig.slot0.kP = Constants.SWERVE.DRIVE_KP;
        swerveDriveFXConfig.slot0.kI = Constants.SWERVE.DRIVE_KI;
        swerveDriveFXConfig.slot0.kD = Constants.SWERVE.DRIVE_KD;
        swerveDriveFXConfig.slot0.kF = Constants.SWERVE.DRIVE_KF;
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;

        swerveCANCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCANCoderConfig.sensorDirection = Constants.SWERVE.CANCODER_INVERT;
        swerveCANCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCANCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;

    }

}
