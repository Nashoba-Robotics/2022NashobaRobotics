package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimberMotor;

//Climbing Command
public class RetractClimberCommand extends CommandBase {
    private double lPos;
    private double rPos;
    public RetractClimberCommand(){
        addRequirements(ClimberSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        ClimberSubsystem.getInstance().undeployClimber();
        // SmartDashboard.putNumber("LC Stator", ClimberSubsystem.getInstance().getStatorCurrent(ClimberMotor.LEFT_CLIMBER));
        // SmartDashboard.putNumber("RC Stator", ClimberSubsystem.getInstance().getStatorCurrent(ClimberMotor.RIGHT_CLIMBER));
    }

    @Override
    public void execute() {
        lPos = ClimberSubsystem.getInstance().getPosition(ClimberMotor.LEFT_CLIMBER);
        rPos = ClimberSubsystem.getInstance().getPosition(ClimberMotor.RIGHT_CLIMBER);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(lPos - Constants.Climber.RETRACT_LEFT_POS) <= Constants.Climber.CLIMB_DEADZONE||
               Math.abs(rPos - Constants.Climber.RETRACT_RIGHT_POS) <= Constants.Climber.CLIMB_DEADZONE;
    }

    @Override
    public void end(boolean interrupted) {
        // SmartDashboard.putNumber("L 1 Pos", ClimberSubsystem.getInstance().getPosition(ClimberMotor.LEFT_1));
        // SmartDashboard.putNumber("R 1 Pos", ClimberSubsystem.getInstance().getPosition(ClimberMotor.RIGHT_1));
    }
}
