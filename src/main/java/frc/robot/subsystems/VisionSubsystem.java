package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    PhotonCamera camera;

    private double pitch = 0.0;
    private double yaw = 0.0;
    private double area = 0.0;
    private boolean validTargets = false;

    public VisionSubsystem() {
        camera = new PhotonCamera("photonvision");
    }

    @Override
    public void periodic() {
        validTargets = false;
        PhotonPipelineResult result = camera.getLatestResult();
        if ( result.hasTargets() ) {
            validTargets = true;
            PhotonTrackedTarget target = result.getBestTarget();
            pitch = target.getPitch();
            yaw = target.getYaw();
            area = target.getArea();
       }
    }

    public boolean hasValidTargets() {
        return validTargets;
    }

    public double getPitch() {
        return pitch;
    }

    public double getYaw() {
        return yaw;
    }

    public double getArea() {
        return area;
    }
}
