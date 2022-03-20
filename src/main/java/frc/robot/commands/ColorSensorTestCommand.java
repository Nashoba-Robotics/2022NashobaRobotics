package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.ColorDetection;
import frc.robot.lib.ColorDetection.BallColor;
import frc.robot.lib.PicoColorSensor.RawColor;
import frc.robot.subsystems.ColorSensorSubsystem;

public class ColorSensorTestCommand extends CommandBase {
    // private ColorDetection cd;

    public void initialize() {
        // cd = new ColorDetection();
    }

    public void execute() {
        // cd.addData(ColorSensorSubsystem.getInstance().getSensor1());
        // cd.addData(ColorSensorSubsystem.getInstance().getSensor2());
        // RawColor rc = ColorSensorSubsystem.getInstance().getSensor1();
        // SmartDashboard.putNumber("DRed", ColorDetection.getDRed(rc));
        // SmartDashboard.putNumber("DBlue", ColorDetection.getDBlue(rc));
        //BallColor c = ColorDetection.detect(rc);
        //SmartDashboard.putString("Ball color", c.toString());
    }

    public void end(boolean interrupted) {
        // try {
        //     SmartDashboard.putString("Data", cd.get().toString());
        // } catch(Exception e) {
        //     SmartDashboard.putString("Data", e.toString());
        // }
    }
}
