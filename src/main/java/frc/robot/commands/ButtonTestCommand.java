package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.JoystickSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ButtonTestCommand extends CommandBase{
    private int buttonID;

    private JoystickButton[] everyDamnButton = {
        new JoystickButton(JoystickSubsystem.getInstance().getLeftJoystick(), 1),
        new JoystickButton(JoystickSubsystem.getInstance().getLeftJoystick(), 2),
        new JoystickButton(JoystickSubsystem.getInstance().getLeftJoystick(), 3),
        new JoystickButton(JoystickSubsystem.getInstance().getLeftJoystick(), 4),
        new JoystickButton(JoystickSubsystem.getInstance().getLeftJoystick(), 5),
        new JoystickButton(JoystickSubsystem.getInstance().getLeftJoystick(), 6),
        new JoystickButton(JoystickSubsystem.getInstance().getLeftJoystick(), 7),
        new JoystickButton(JoystickSubsystem.getInstance().getLeftJoystick(), 8),
        new JoystickButton(JoystickSubsystem.getInstance().getLeftJoystick(), 9),
        new JoystickButton(JoystickSubsystem.getInstance().getLeftJoystick(), 10),
        new JoystickButton(JoystickSubsystem.getInstance().getLeftJoystick(), 11),
        new JoystickButton(JoystickSubsystem.getInstance().getLeftJoystick(), 12),
        new JoystickButton(JoystickSubsystem.getInstance().getLeftJoystick(), 13),
        new JoystickButton(JoystickSubsystem.getInstance().getLeftJoystick(), 14),
        new JoystickButton(JoystickSubsystem.getInstance().getLeftJoystick(), 15),
        new JoystickButton(JoystickSubsystem.getInstance().getLeftJoystick(), 16),
    };

    private JoystickButton[] combination = {
        everyDamnButton[7],
        everyDamnButton[9],
        everyDamnButton[10],
        everyDamnButton[12],
        everyDamnButton[5],
        everyDamnButton[8]
    };   //1 5 11 7 2 3

    private boolean unlocked;

    int index;
    JoystickButton joystickButton;
    JoystickButton previousButton;
    Boolean previousOutput;
    public ButtonTestCommand(){
        addRequirements(JoystickSubsystem.getInstance());
        buttonID = 8;
        SmartDashboard.putNumber("Button ID", buttonID);
        joystickButton = new JoystickButton(JoystickSubsystem.getInstance().getLeftJoystick(), buttonID);
        SmartDashboard.putBoolean("Unlocked", false);
        SmartDashboard.putBoolean("Incorrect Number", true);
        unlocked = false;
    }
    @Override
    public void execute(){
        SmartDashboard.putBoolean("Incorrect Number", true);
        if(combination[index].get()){
            previousButton = combination[index];
            previousOutput = true;
            index++;
        }
        else{
            for(int i = 0; i < 16; i++){
                if(everyDamnButton[i].get() && !unlocked && !previousOutput){
                    index = 0;
                    SmartDashboard.putBoolean("Incorrect Number", false);
                    continue;
                }
            }
        }
        if(previousButton != null){
            if(!previousButton.get()){
                previousButton = null;
                previousOutput = false;
            }
        }
        if(index >= combination.length){
            SmartDashboard.putBoolean("Unlocked", true);
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}