package frc.robot.commands.intakeshoot;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoAimMotionMagicCommand;
import frc.robot.commands.JoystickDriveCommand;

public class AimShootCG extends SequentialCommandGroup {
    public AimShootCG(){
        addCommands(
            new AutoAimMotionMagicCommand(true),
            new ParallelCommandGroup(
                new ShootCommand(true), 
                new JoystickDriveCommand()
            )
        );
    }
    
}
