package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class SwerveDriveCommand extends CommandBase {

    private Drivetrain m_drive;
    private XboxController m_controller;

    // // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to
    // 1.
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(2);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(2);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(2);

    private final double m_deadBand = 0.1;

    public SwerveDriveCommand(Drivetrain drive, XboxController controller) {
        m_drive = drive;
        m_controller = controller;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        driveWithJoystick(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0, 0, 0, false);
    }

    private void driveWithJoystick(boolean fieldRelative) {
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        final var xSpeed = -m_xspeedLimiter.calculate(deadband(m_controller.getLeftY(), m_deadBand)) * Drivetrain.kMaxSpeed;

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        final var ySpeed = -m_yspeedLimiter.calculate(deadband(m_controller.getLeftX(), m_deadBand)) * Drivetrain.kMaxSpeed;

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        final var rot = -m_rotLimiter.calculate(deadband(m_controller.getRightX(), m_deadBand)) * Drivetrain.kMaxAngularSpeed;

        m_drive.drive(xSpeed, ySpeed, rot, fieldRelative);
    }

    public static double deadband(double input, double deadband) {
        if (Math.abs(input) < deadband) {
            return 0.0;
        }
        return input;
    }


}
