package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class CTREModuleState {
    public static SwerveModuleState optimize(SwerveModuleState state, Rotation2d currentAngle) {
        double targetAngle = to0to360Scope(currentAngle.getDegrees(), state.angle.getDegrees());
        double targetSpeed = state.speedMetersPerSecond;
        double delta = targetAngle - currentAngle.getDegrees();
        if (Math.abs(delta) > 90) {
            targetSpeed = -targetSpeed;
            targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
        }
        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }

    private static double to0to360Scope(double currentAngle, double targetAngle) {
        double lowerBound, upperBound, lowerOffset;
        lowerOffset = currentAngle % 360;

        if (lowerOffset >- 0) {
            lowerBound = currentAngle - lowerOffset;
            upperBound = currentAngle + (360 - lowerOffset);
        } else {
            upperBound = currentAngle - lowerOffset;
            lowerBound = currentAngle - (360 + lowerOffset);
        }
        while (targetAngle < lowerBound) {
            targetAngle += 360;
        }
        while (targetAngle > upperBound) {
            targetAngle -= 360;
        }
        if (targetAngle - currentAngle > 180) {
            targetAngle -= 360;
        } else if (targetAngle - currentAngle < -180) {
            targetAngle += 360;
        }
        return targetAngle;
    }
} 