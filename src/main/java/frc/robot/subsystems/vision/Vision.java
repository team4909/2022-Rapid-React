package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    //#region Constants
    private final double LIMELIGHT_HEIGHT_METERS = Units.inchesToMeters(42.5);
    private final double GOAL_HEIGHT_METERS = Units.inchesToMeters(103);
    private final double LIMELIGHT_PITCH_RADIANS = Units.degreesToRadians(20);
    //#endregion

    private static Vision m_instance = null; 
    private PhotonPipelineResult m_pipelineResult;

    PhotonCamera limelight;
    private Vision() {
        limelight = new PhotonCamera(NetworkTableInstance.getDefault(), "limelight");
        SmartDashboard.putNumber("Photon Distance", 0);
    }

    @Override
    public void periodic() {
        m_pipelineResult = limelight.getLatestResult();

        if (m_pipelineResult.hasTargets()) {
            double targetPitch = Units.degreesToRadians(m_pipelineResult.getBestTarget().getPitch());

            double distance = PhotonUtils.calculateDistanceToTargetMeters(
                LIMELIGHT_HEIGHT_METERS,
                GOAL_HEIGHT_METERS,
                LIMELIGHT_PITCH_RADIANS,
                targetPitch);

            SmartDashboard.putNumber("Photon Distance", distance);
        }
    }

    public void takeSnapshot() {
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
