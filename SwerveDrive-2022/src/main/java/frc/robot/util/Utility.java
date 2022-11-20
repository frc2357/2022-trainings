package frc.robot.util;

public class Utility {
    
    public static double deadband(double input, double deadband) {
        if (Math.abs(input) < deadband) {
            return 0.0;
        }
        return input;
    }

    public static double modifyAxis(double value, double db) {
        value = deadband(value, db);

        value = Math.copySign(value * value, value);

        return value;
    }
}
