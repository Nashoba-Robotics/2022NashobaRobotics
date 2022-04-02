package frc.robot.commands;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LedSubsystem;

public class LedTestCommand extends CommandBase {
    SendableChooser<Animation> animationChooser;
    
    public LedTestCommand() {
        addRequirements(LedSubsystem.getInstance());
        // animationChooser = new SendableChooser<>();
        // animationChooser.addOption("colorflow", new ColorFlowAnimation(255, 0, 0, 0, 0.5, 46, Direction.Forward));
        // animationChooser.addOption("fire", new FireAnimation(1, .5, 46, 0.5, 0.5));
        // animationChooser.addOption("larson", new LarsonAnimation(255, 0, 0, 0, 0.5, 46, BounceMode.Center, 5));
        // animationChooser.addOption("rgbfade", new RgbFadeAnimation(1, .5, 46));
        // animationChooser.addOption("singlefade", new SingleFadeAnimation(255, 0, 0, 0, 0.5, 46));
        // animationChooser.addOption("twinkle", new TwinkleAnimation(255, 0, 0, 0, .5, 46, TwinklePercent.Percent64));
        // animationChooser.addOption("twinkleoff", new TwinkleOffAnimation(255, 0, 0, 0, 0.5, 46, TwinkleOffPercent.Percent64));
    }

    public void initialize() {
        // SmartDashboard.putData(animationChooser);
        SmartDashboard.putNumber("R", 0);
        SmartDashboard.putNumber("G", 0);
        SmartDashboard.putNumber("B", 0);
    }

    public void execute() {
        int[] col = new int[]{
            (int)SmartDashboard.getNumber("R", 0),
            (int)SmartDashboard.getNumber("G", 0),
            (int)SmartDashboard.getNumber("B", 0),
        };
        LedSubsystem.getInstance().twinkle(col);
        //LedSubsystem.getInstance().setAnimation(animationChooser.getSelected());
    }
}
