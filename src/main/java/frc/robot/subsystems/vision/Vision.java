package frc.robot.subsystems.vision;

import edu.wpi.first.math.util.Units;

public class Vision {

	private static Vision m_instance = null;
	//#endregion
	//#region Constants
	private final double LIMELIGHT_HEIGHT_METERS = Units.inchesToMeters(0);

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
