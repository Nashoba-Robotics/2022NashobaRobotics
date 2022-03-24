package frc.robot.commands.intakeshoot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.lib.Units;
import frc.robot.subsystems.ColorSensorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LoaderSubsystem;
import frc.robot.lib.ColorDetection.BallColor;

public class SetColorRejectionCommand extends CommandBase {
    boolean on;

    public SetColorRejectionCommand(boolean on) {
        this.on = on;
    }
    
    @Override
    public void execute() {
        RunIntakeCommand.setColorRejection(on);
    }

    @Override
    public boolean isFinished() {
       return true;
    }
}  
