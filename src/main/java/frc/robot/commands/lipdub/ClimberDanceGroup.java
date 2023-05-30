package frc.robot.commands.lipdub;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberDanceGroup extends SequentialCommandGroup {
    
    public ClimberDanceGroup() {
        addCommands(
            new ClimberUp(),
            new ClimberDown()  
        );
    }

}
