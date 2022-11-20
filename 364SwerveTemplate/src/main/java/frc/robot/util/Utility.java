package frc.robot.util;

public class Utility {

    public static double deadband(double input, double deadband) {
        if (Math.abs(input) < deadband) {
            return 0.0;
        }
        return input;
    }

}