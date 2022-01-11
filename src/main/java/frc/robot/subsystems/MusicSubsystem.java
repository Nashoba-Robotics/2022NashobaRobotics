package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.sendable.SendableRegistry;

import frc.robot.Constants;
import frc.robot.commands.StopCommand;
import frc.robot.lib.Units;

public class MusicSubsystem extends SubsystemBase {

    public TalonFX musicMotor;
    
    private static MusicSubsystem instance;

    private MusicSubsystem() {
        musicMotor = new TalonFX(Constants.WINCH_PORT);

    }

    public static MusicSubsystem getInstance() {
        if(instance == null) {
            instance = new MusicSubsystem();
        }
        return instance;
    }

    public void setFrequency(double frequency){
        musicMotor.set(ControlMode.MusicTone, frequency);
    }
}
