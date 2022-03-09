package frc.robot.commands;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ColorSensorCommand extends CommandBase{

    private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a 
   * parameter. The device will be automatically initialized with default 
   * parameters.
   */
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  /**
   * A Rev Color Match object is used to register and detect known colors. This can 
   * be calibrated ahead of time or during operation.
   * 
   * This object uses a simple euclidian distance to estimate the closest match
   * with given confidence range.
   */
  private final ColorMatch m_colorMatcher = new ColorMatch();

  /**
   * Note: Any example colors should be calibrated as the user needs, these
   * are here as a basic example.
   */
  private final Color kBlueTarget = new Color(0.26, 0.47, 0.27);
  private final Color kRedTarget = new Color(0.35, 0.45, 0.214);
  private final Color kNoneTarget = new Color(0.3,0.47,0.2);

    public void initialize(){
        m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
        m_chooser.addOption("My Auto", kCustomAuto);
        SmartDashboard.putData("Auto choices", m_chooser);

        m_colorMatcher.addColorMatch(kBlueTarget);
        m_colorMatcher.addColorMatch(kRedTarget);
        m_colorMatcher.addColorMatch(kNoneTarget);
    }
    public void execute(){
        
        // Copyright (c) FIRST and other WPILib contributors.
        // Open Source Software; you can modify and/or share it under the terms of
        // the WPILib BSD license file in the root directory of this project.

        /**
         * The VM is configured to automatically run this class, and to call the functions corresponding to
         * each mode, as described in the TimedRobot documentation. If you change the name of this class or
         * the package after creating this project, you must also update the build.gradle file in the
         * project.
         */
        

        /**
         * This function is run when the robot is first started up and should be used for any
         * initialization code.
         */

        /**
         * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
         * that you want ran during disabled, autonomous, teleoperated and test.
         *
         * <p>This runs after the mode specific periodic functions, but before LiveWindow and
         * SmartDashboard integrated updating.
         */

        Color detectedColor = m_colorSensor.getColor();

        /**
         * Run the color match algorithm on our detected color
         */
        String colorString;
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

        if (match.color == kBlueTarget) {
        colorString = "Blue";
        } else if (match.color == kRedTarget) {
        colorString = "Red";
        }else if (match.color == kNoneTarget){
        colorString = "None";
        
        } else {
        colorString = "Unknown";
        }

        /**
         * Open Smart Dashboard or Shuffleboard to see the color detected by the 
         * sensor.
         */
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("Confidence", match.confidence);
        SmartDashboard.putString("Detected Color", colorString);

  

    }
    public boolean isFinished(){
        return true;
    }
    public void end(boolean interrupted){

    }
}
