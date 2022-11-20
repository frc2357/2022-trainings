package frc.robot.controls;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.util.Utility;

public class DriveControls {
    protected XboxController m_controller;
    private double m_deadband;

    public DriveControls(XboxController controller, double deadband) {
        m_controller = controller;
        m_deadband = deadband;
    }

    // TODO: Switch to TriggerAxis
    public double getX() {
        return -Utility.modifyAxis(m_controller.getLeftX(), m_deadband);
    }

    public double getY() {
        return -Utility.modifyAxis(m_controller.getLeftY(), m_deadband);
    }

    public double getRotation() {
        return -Utility.modifyAxis(m_controller.getRightX(), m_deadband);
    }
    
}
