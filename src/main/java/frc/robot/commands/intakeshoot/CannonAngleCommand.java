package frc.robot.commands.intakeshoot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CannonSubsystem;
import frc.robot.subsystems.CannonSubsystem.Angle;

public class CannonAngleCommand extends CommandBase {
    Angle angle;

    public CannonAngleCommand(Angle angle) {
        this.angle = angle;
    }

    @Override
    public void initialize() {
        CannonSubsystem.getInstance().setAngle(angle);
        SmartDashboard.putBoolean("Shooter Solenoid 80?", angle == Angle.EIGHTY);
    }

    @Override
    public void execute() { 
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
       return true;
    }
}  
