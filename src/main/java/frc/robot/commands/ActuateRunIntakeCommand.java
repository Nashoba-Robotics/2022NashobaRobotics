package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.intakeshoot.RunIntakeCommand;
import frc.robot.commands.intakeshoot.StopIntakeCommand;
import frc.robot.subsystems.IntakeSolenoidSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ActuateRunIntakeCommand extends SequentialCommandGroup {
    public ActuateRunIntakeCommand(){
        addCommands(
            new ActuateIntakeCommand(true),
            new RunIntakeCommand().until(RobotContainer::getSensor2),
            new StopIntakeCommand().withTimeout(2),
            new ActuateIntakeCommand(false)
        );
    }

}
