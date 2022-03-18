package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.util.Units;

public class Vision {

    //#region Constants
    private final double LIMELIGHT_HEIGHT_METERS = Units.inchesToMeters(0);
    //#endregion

    private static Vision m_instance = null; 

    // PhotonCamera cam = ;
    private Vision() {

    }

    public static Vision getInstance() {
        if (m_instance == null) {
            m_instance = new Vision();
        }

        return m_instance;
    }
}
