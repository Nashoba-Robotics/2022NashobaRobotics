package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.ColorDetection;
import frc.robot.lib.PicoColorSensor.RawColor;
import frc.robot.subsystems.ColorSensorSubsystem;

public class ColorSensorTestCommand extends CommandBase {
    double r;
    double g;
    double b;
    double ir;
    int count;

    public void initialize() {
       r = 0; g = 0; b = 0; ir = 0; count = 0;
    }

    public void execute() {

        SmartDashboard.putString("Ball", ColorSensorSubsystem.getInstance().getBall().toString());
        RawColor c1 = ColorSensorSubsystem.getInstance().getSensor1();
        RawColor c2 = ColorSensorSubsystem.getInstance().getSensor2();
        r += c1.red + c2.red;
        g += c1.green + c2.green;
        b += c1.blue + c2.blue;
        ir += c1.ir + c2.ir;
        count += 2;
    }

    public void end(boolean interrupted) {
        double[] data = new double[]{r,g,b,ir};
        double[] norm = ColorDetection.normalize(data);
        SmartDashboard.putNumber("color R", norm[0]);
        SmartDashboard.putNumber("color G", norm[1]);
        SmartDashboard.putNumber("color B", norm[2]);
        SmartDashboard.putNumber("color IR", norm[3]);
    }
}
