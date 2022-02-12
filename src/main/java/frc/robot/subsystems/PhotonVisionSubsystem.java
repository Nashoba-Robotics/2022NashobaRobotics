package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PhotonVisionSubsystem extends SubsystemBase{
    private static PhotonVisionSubsystem instance;
    private PhotonCamera camera;
    private PhotonPipelineResult result;

    public PhotonVisionSubsystem() {
        camera = new PhotonCamera(Constants.PHOTONVISION_NICKNAME);
    }

    public static PhotonVisionSubsystem getInstance() {
        if(instance == null) {
            instance = new PhotonVisionSubsystem();
        }
        return instance;
    }

    public void update() {
        result = camera.getLatestResult();
    }

    public boolean validTarget() {
        return result.hasTargets();
    }

    public double getTx() {
        return result.getBestTarget().getYaw();
    }

    public double getTy() {
        return result.getBestTarget().getPitch();
    }

    public void setLed(VisionLEDMode led) {
        camera.setLED(led);
    }

}
