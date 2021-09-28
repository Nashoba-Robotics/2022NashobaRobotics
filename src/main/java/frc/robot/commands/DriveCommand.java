package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.JoystickProcessing;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.JoystickSubsystem;

public class DriveCommand extends CommandBase {
    public DriveCommand() {
        addRequirements(DriveSubsystem.getInstance());
        addRequirements(JoystickSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double rightX = JoystickSubsystem.getInstance().getRightX();
        double leftY = JoystickSubsystem.getInstance().getLeftY();
        double[] speeds = JoystickProcessing.processJoysticks(rightX, leftY);
        DriveSubsystem.getInstance().setSpeed(-speeds[0], speeds[1]);
        SmartDashboard.putNumber("Left speed", speeds[0]);
        SmartDashboard.putNumber("Right speed", speeds[1]);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
