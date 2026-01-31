package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveCommand extends Command {
    private final DrivetrainSubsystem m_drivetrain;
    private final XboxController m_controller;

    public DriveCommand(DrivetrainSubsystem drivetrain, XboxController controller) {
        m_drivetrain = drivetrain;
        m_controller = controller;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        // Slow mode toggle
        double speedMultiplier = m_controller.getRightBumper() ? 0.5 : 1.0;

        // Get joystick inputs
        // The Y acts as forward/backward (X in field coordinates)
        // The X acts as left/right (Y in field coordinates)
        // We invert Y because pushing the stick forward returns a negative value
        double xSpeed = -MathUtil.applyDeadband(m_controller.getLeftY(), OIConstants.DRIVE_DEADBAND) 
            * DriveConstants.MAX_SPEED_METERS_PER_SECOND * speedMultiplier;
            
        double ySpeed = -MathUtil.applyDeadband(m_controller.getLeftX(), OIConstants.DRIVE_DEADBAND) 
            * DriveConstants.MAX_SPEED_METERS_PER_SECOND * speedMultiplier;
            
        double rot = -MathUtil.applyDeadband(m_controller.getRightX(), OIConstants.DRIVE_DEADBAND) 
            * DriveConstants.MAX_ANGULAR_SPEED * speedMultiplier;

        m_drivetrain.drive(xSpeed, ySpeed, rot, true);
    }
}
