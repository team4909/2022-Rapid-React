package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.util.Units;

public class Vision {

    //#region Constants
    private final double LIMELIGHT_HEIGHT_METERS = Units.inchesToMeters(inches)
    //#endregion

    private static Vision m_instance = null; 

    PhotonCamera cam = STUPID FUCKING CAMERA;
    private Vision() {

    }

    public static Vision getInstance() {
        if (m_instance == null) {
            m_instance = new Vision();
        }

        return m_instance;
    }
}
