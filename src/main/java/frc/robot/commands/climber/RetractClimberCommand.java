package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.LedSubsystem;

//Climbing Command
public class RetractClimberCommand extends CommandBase {
    private double lPos;
    private double rPos;
    public RetractClimberCommand(){
        addRequirements(ClimberSubsystem.getInstance());
        addRequirements(LedSubsystem.getInstance()); //Uses LEDs in the TraversalClimbCommand
                                                     //Ben said I was supposed to put it here
                                                     //If anyone is mad at this, blame Ben
    }

    @Override
    public void initialize() {
        ClimberSubsystem.getInstance().undeployClimber();
        LedSubsystem.getInstance().twinkle(Constants.Leds.TRAVERSAL_CLIMB);
    }

    @Override
    public boolean isFinished() {
        lPos = ClimberSubsystem.getInstance().getLeftPosition();
        rPos = ClimberSubsystem.getInstance().getRightPosition();
        return Math.abs(lPos - Constants.Climber.RETRACT_LEFT_POS) <= Constants.Climber.CLIMB_DEADZONE||
               Math.abs(rPos - Constants.Climber.RETRACT_RIGHT_POS) <= Constants.Climber.CLIMB_DEADZONE;
    }
}
