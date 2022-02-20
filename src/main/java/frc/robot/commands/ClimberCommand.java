package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Climber;
import frc.robot.subsystems.CannonSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimberMotor;

/*
    Command to test the cannon. Allows the user to input a percent into Shuffleboard
    to use for the cannon velocity
*/
public class ClimberCommand extends CommandBase {
    boolean finished;

    public ClimberCommand(){
        addRequirements(ClimberSubsystem.getInstance());
    }

    @Override
    public void initialize(){
        finished = false;
        SmartDashboard.putNumber("L 1", 0);
        SmartDashboard.putNumber("L 2", 0);
        SmartDashboard.putNumber("L Rotate", 0);
    }

    @Override
    public void execute(){
        ClimberSubsystem csi = ClimberSubsystem.getInstance();
        if(csi.isCurrentBad()) {
            csi.stop();
            finished = true;
            return;
        }
        csi.setSpeed(ClimberMotor.LEFT_1, SmartDashboard.getNumber("L 1",0));
        csi.setSpeed(ClimberMotor.LEFT_2, SmartDashboard.getNumber("L 2",0));
        csi.setSpeed(ClimberMotor.LEFT_ROTATE, SmartDashboard.getNumber("L Rotate",0));
        SmartDashboard.putNumber("L 1 Stator Current", csi.getStatorCurrent(motor))
    }

    @Override
    public void end(boolean interrupted){
        ClimberSubsystem.getInstance().stop();
        finished = true;
    }

    @Override
    public boolean isFinished(){
        return finished;
    }
}