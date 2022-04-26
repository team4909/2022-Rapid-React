package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {

    //#region Constants
    private final double LIMELIGHT_HEIGHT_METERS = Units.inchesToMeters(42.5);
    private final double GOAL_HEIGHT_METERS = Units.inchesToMeters(103);
    private final double LIMELIGHT_PITCH_RADIANS = Units.degreesToRadians(15);
    //#endregion

    private static Vision m_instance = null; 
    private PhotonPipelineResult m_pipelineResult;

    private boolean m_isAligned;
    private double m_xOffset;
    private double m_distance;
    private double m_avgDistance;

    private double limelightOffset;
    private PIDController m_turnPID;

    PhotonCamera limelight;

    LinearFilter m_distanceFilter;
    LinearFilter m_offsetFilter;

    private Vision() {
        limelight = new PhotonCamera(NetworkTableInstance.getDefault(), "Limelight");
        // SmartDashboard.putNumber("Photon Distance", 0);
        m_turnPID = 
            new PIDController(VisionConstants.kVisionPIDGains.kP, VisionConstants.kVisionPIDGains.kI, VisionConstants.kVisionPIDGains.kD);

        // m_distanceFilter = LinearFilter.singlePoleIIR(.1, 0.02);
        // m_offsetFilter = LinearFilter.singlePoleIIR(.1, 0.02);
 
        // BB Try this
        m_distanceFilter = LinearFilter.movingAverage(10);
        m_offsetFilter = LinearFilter.movingAverage(10);
        SmartDashboard.putData(m_turnPID);
    }


    @Override
    public void periodic() {
        m_pipelineResult = limelight.getLatestResult();

        // SmartDashboard.putNumber("error", m_turnPID.getVelocityError());

        if (m_pipelineResult.hasTargets()) {
            double targetPitch = Units.degreesToRadians(m_pipelineResult.getBestTarget().getPitch());

            m_distance = PhotonUtils.calculateDistanceToTargetMeters(
                LIMELIGHT_HEIGHT_METERS,
                GOAL_HEIGHT_METERS,
                LIMELIGHT_PITCH_RADIANS,
                targetPitch);

            m_xOffset = m_offsetFilter.calculate(m_pipelineResult.getBestTarget().getYaw());
            SmartDashboard.putBoolean("Is Aligned", m_isAligned);

            SmartDashboard.putNumber("Photon Distance", m_distance);
            m_avgDistance = m_distanceFilter.calculate(m_distance);

            
            SmartDashboard.putNumber("Photon Distance avg", m_avgDistance);

        } else {
            m_xOffset = 0;
            m_distance = 0;
        }

        if (Math.abs(this.m_xOffset) <= 1) {
            m_isAligned = true;
        } else {
            m_isAligned = false;
        }
    }

    public void setLimelightOffset() {

        double offset = -this.m_xOffset;

        // Use a PID to convert between a offset yaw degrees to an angular speed for robot rotation
        double angularSpeed = -m_turnPID.calculate(offset, 0);
        // double angularSpeed = (offset * Constants.GOAL_ALIGN_KP + Math.abs(offset - firstError) * Constants.GOAL_ALIGN_KD) * 2;
        SmartDashboard.putNumber("Angular Speed", angularSpeed);
        SmartDashboard.putNumber("Offset", offset);
        if (this.m_isAligned == true) {
            limelightOffset = 0;
        } else {
            if (this.m_xOffset <= 0)
                limelightOffset = angularSpeed + VisionConstants.kVisionPIDGains.kF;

            if (this.m_xOffset >= 0) 
                limelightOffset = angularSpeed - VisionConstants.kVisionPIDGains.kF;
        }

    }

    public void setLimelightOffset(double value) {
        limelightOffset = value;
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

    public double getDistance() {
        return m_distance;
    }

    public double getAverageDistance(){
        return m_avgDistance;
    }

    public double getLimelightOffset() {
        return this.limelightOffset;
    }

    public static Vision getInstance() {
        if (m_instance == null) {
            m_instance = new Vision();
        }

        return m_instance;
    }
    
}
