package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    //#region Constants
    private final double LIMELIGHT_HEIGHT_METERS = Units.inchesToMeters(0);
    private final double GOAL_HEIGHT_METERS = Units.inchesToMeters(0);
    //#endregion

    private static Vision m_instance = null; 
    private PhotonPipelineResult m_pipelineResult;

    PhotonCamera limelight;
    private Vision() {

    }

    @Override
    public void periodic() {
        m_pipelineResult = limelight.getLatestResult();

        if (m_pipelineResult.hasTargets()) {
            
        }
    }

    private void takeSnapshot() {
        limelight.takeInputSnapshot();
        limelight.takeOutputSnapshot();
    }

    /**
     * @return seconds
     */
    public double getLatency() {
        return m_pipelineResult.getLatencyMillis() / 1000d;
    }


    public static Vision getInstance() {
        if (m_instance == null) {
            m_instance = new Vision();
        }

        return m_instance;
    }
    
}
