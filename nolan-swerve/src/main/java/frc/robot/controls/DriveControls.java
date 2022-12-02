package frc.robot.controls;

import edu.wpi.first.wpilibj.XboxController;

public class DriveControls {
    protected XboxController m_controller;
    private double m_deadband;

    public DriveControls(XboxController controller, double deadband) {
        m_controller = controller;
        m_deadband = deadband;
    }

    // TODO: Switch to TriggerAxis
    public double getX() {
        return -modifyAxis(m_controller.getLeftX(), m_deadband);
    }

    public double getY() {
        return modifyAxis(m_controller.getLeftY(), m_deadband);
    }

    public double getRotation() {
        return -modifyAxis(m_controller.getRightX(), m_deadband);
    }

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
