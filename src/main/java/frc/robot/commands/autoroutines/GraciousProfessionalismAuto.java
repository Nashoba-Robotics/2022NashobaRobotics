package frc.robot.commands.autoroutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DoNothingAutoCommand;

public class GraciousProfessionalismAuto extends SequentialCommandGroup{
    public GraciousProfessionalismAuto(){
        addCommands(
            new DoNothingAutoCommand()
        );
    }
}
