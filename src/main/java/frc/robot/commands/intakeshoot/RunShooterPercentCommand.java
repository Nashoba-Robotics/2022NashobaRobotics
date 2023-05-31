package frc.robot.commands.intakeshoot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Cannon;
import frc.robot.subsystems.CannonSubsystem;
import frc.robot.subsystems.CannonSubsystem.Angle;

public class RunShooterPercentCommand extends CommandBase{
    
    public RunShooterPercentCommand() {
        addRequirements(CannonSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Cannon Speed", 0.5);
        CannonSubsystem.getInstance().setAngle(Angle.SIXTY);
    }

    @Override
    public void execute() {
        CannonSubsystem.getInstance().set(SmartDashboard.getNumber("Cannon Speed", 0.5));
    }

    @Override
    public void end(boolean interrupted) {
        CannonSubsystem.getInstance().set(0);
    }    

}
