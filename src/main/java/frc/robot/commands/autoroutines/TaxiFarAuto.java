package frc.robot.commands.autoroutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.AutoPaths;
import frc.robot.commands.AutoShootCommand;
import frc.robot.subsystems.CannonSubsystem.Angle;

public class TaxiFarAuto extends SequentialCommandGroup{
    public TaxiFarAuto(){
        addCommands(
            new AutoShootCommand(Angle.SIXTY),
            new WaitCommand(5),
            new PathFollowCommand(AutoPaths.TWO_BALL_FAR_AUTO)
        );
    }
}
