package frc.robot.util;

public class SwerveModuleConstants {
    public final int DRIVE_MOTOR_ID;
    public final int ANGLE_MOTOR_ID;
    public final int CANCODER_ID;
    public final double ANGLE_OFFSET;

    public SwerveModuleConstants(int DRIVE_MOTOR_ID, int ANGLE_MOTOR_ID, int CANCODER_ID, double ANGLE_OFFSET) {
        this.DRIVE_MOTOR_ID = DRIVE_MOTOR_ID;
        this.ANGLE_MOTOR_ID = ANGLE_MOTOR_ID;
        this.CANCODER_ID = CANCODER_ID;
        this.ANGLE_OFFSET = ANGLE_OFFSET;
        
    }
}