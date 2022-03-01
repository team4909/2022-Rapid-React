package frc.robot.subsystems.vision;

import java.util.Map;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class EnableClimberCamera {
    
    private UsbCamera climberCamera_;
    
    public EnableClimberCamera(boolean enable) {
        if (enable) {
            
            climberCamera_ = CameraServer.startAutomaticCapture("Climber Camera", 1);
            climberCamera_.setResolution(240, 128);
            climberCamera_.setFPS(30);

            Shuffleboard.getTab("Driver").add(climberCamera_)
            .withPosition(8, 4)
            .withSize(5, 2)
            .withWidget("Camera Stream")
            .withProperties(Map.of("Rotation", "HALF", "Show controls", true));
        
        } else {
            climberCamera_.close();
        }
    }
}
