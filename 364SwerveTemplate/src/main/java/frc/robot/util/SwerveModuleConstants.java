package frc.robot.util;

public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int turnMotorID;
    public final int cancoderID;
    public final double angleOffset;

    public SwerveModuleConstants(int driveMotorID, int turnMotorID, int cancoderID, double angleOffset) {
        this.driveMotorID = driveMotorID;
        this.turnMotorID = turnMotorID;
        this.cancoderID = cancoderID;
        this.angleOffset = angleOffset;
    }
}

