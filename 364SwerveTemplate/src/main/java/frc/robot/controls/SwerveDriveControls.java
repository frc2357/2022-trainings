package frc.robot.controls;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.util.Utility;

public class SwerveDriveControls {
    
    private XboxController m_controller;
    private double m_deadband;

    public SwerveDriveControls(XboxController controller, double deadband) {
        m_controller = controller;
        m_deadband = deadband;
    }

    public double getTranslation() {
        double value = m_controller.getLeftY();
        return Utility.deadband(value, m_deadband);
    }

    public double getStrafe() {
        double value = m_controller.getLeftX();
        return Utility.deadband(value, m_deadband);
    }

    public double getRotation() {
        double value = m_controller.getRightX();
        return Utility.deadband(value, m_deadband);
    }

}
