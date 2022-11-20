package frc.robot.util;

public class Conversions {
    
    public static double falconToDegrees(double counts, double gearRatio) {
        return counts * (360.0 / (gearRatio * 2048.0));
    }

    public static double degreesToFalcon(double degrees, double gearRatio) {
        return degrees / (360.0 / (gearRatio * 2048.0));
    }

    public static double falconToRPM(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * (600.0 / 2048.0);
        return motorRPM / gearRatio;
    }

    public static double RPMToFalcon(double rpm, double gearRatio) {
        double motorRPM = rpm * gearRatio;
        return motorRPM * (2048.0 / 600.0);
    }

    public static double falconToMPS(double velocityCounts, double circumference, double gearRatio) {
        double wheelRPM = falconToRPM(velocityCounts, gearRatio);
        return (wheelRPM * circumference) / 60;
    }

    public static double MPSToFalcon(double mps, double circumference, double gearRatio) {
        double wheelRPM = ((mps * 60) / circumference);
        return RPMToFalcon(wheelRPM, gearRatio);
    }

}
