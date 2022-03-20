package frc.robot.subsystems.vision;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {
	public static VisionSubsystem instance_ = null;
	private final double firstError;
	private final UsbCamera m_frontCamera;
	private final NetworkTableEntry cameraSelection;
	public boolean isAligned;
	boolean toggle = true;
	// Network table values
	NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
	NetworkTableEntry tx = table.getEntry("tx");
	NetworkTableEntry ty = table.getEntry("ty");
	NetworkTableEntry ta = table.getEntry("ta");
	// Made into privates
	private boolean isAligned_;
	private double lastDistance_;
	private double limelightOffset;
	private UsbCamera climberCamera_;
	private VideoSource currentCamera_;

	private VisionSubsystem() {


		m_frontCamera = CameraServer.startAutomaticCapture(0);
		// m_frontCamera.setResolution(160, 90); //256 144
		m_frontCamera.setVideoMode(PixelFormat.kMJPEG, 640, 480, 30);
		m_frontCamera.setExposureManual(10);
		cameraSelection = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");
		switchCamera("Front Camera");

		isAligned_ = false;
		lastDistance_ = 0.0;
		firstError = this.getXDegrees();


	}

	public static VisionSubsystem getInstance() {
		if (instance_ == null) {
			instance_ = new VisionSubsystem();
		}

		return instance_;
	}

	public void switchCamera(String camName) {
		switch (camName) {
			case "Front Camera":
				cameraSelection.setString(m_frontCamera.getName());
				break;
			case "Climber Camera":
				// currentCamera_ = CameraServer.startAutomaticCapture("Climber Camera", 1);
				break;
			default:
				break;
		}
	}

	public void getLimeLight() {
		double x = tx.getDouble(0.0);
		double y = ty.getDouble(0.0);
		double area = ta.getDouble(0.0);

		SmartDashboard.putNumber("LimeLightX", x);
		SmartDashboard.putNumber("LimeLightY", y);
		SmartDashboard.putNumber("LimeLightArea", area);
	}

	//LED Power is 60

	public double getDistance() {
		return lastDistance_;
	}

	public double getVelocityGoal() {
		double goal;
		// Put these as constants at some point
		double m = 18;
		double b = 2150;
		double y = ty.getDouble(0.0);

		if (y == 0.0) {
			return 0;
		}

		goal = m * getDistance() + b; // Linear equation, goal = mx+b
		SmartDashboard.putNumber("Goal", goal);
		return goal;
	}

	public double getXDegrees() {
		return tx.getDouble(0.0);
	}

	public void setLimelightOffset() {

		double offset = -this.getXDegrees();
		double angularSpeed = (offset * Constants.GOAL_ALIGN_KP + Math.abs(offset - firstError) * Constants.GOAL_ALIGN_KD) * 2; //TODO make this a constant pls

		if (this.isAligned()) { // Todo
			limelightOffset = 0;
		} else {
			if (this.getXDegrees() <= 0)
				limelightOffset = angularSpeed;

			if (this.getXDegrees() >= 0)
				limelightOffset = angularSpeed;
		}

		SmartDashboard.putNumber("Angulat speed", limelightOffset);
	}

	/**
	 * @return chasis speed, not angle in measure
	 */
	public double getLimelightOffset() {
		return this.limelightOffset;
	}

	public void setLimelightOffset(double value) {
		limelightOffset = value;
	}

	public boolean isAligned() {
		return isAligned_;
	}

	public void setPipeline() {
		if (toggle == true) { // ToDo: Keep
			NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
			toggle = false;
		} else {
			NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
			toggle = true;
		}
	}

	public void periodic() {
		// Update the last values
		lastDistance_ = (Constants.tapeHeight - Constants.limelightHeight) / Math.tan(Math.toRadians(Constants.limelightAngle + ty.getDouble(0.0)));
		getLimeLight();

		isAligned_ = Math.abs(getXDegrees()) <= 2;
		SmartDashboard.putBoolean("isAligned", isAligned_);
		SmartDashboard.putNumber("Distance", lastDistance_);
		SmartDashboard.putNumber("ofset speed", limelightOffset);
	}

	public CommandGroupBase LimelightAim() {
		return new RunCommand(this::setLimelightOffset, this).withTimeout(0.5)
				.andThen(() -> this.setLimelightOffset(0));
	}
}
