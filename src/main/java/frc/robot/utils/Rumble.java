package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;

// FixMe: Kelly
public class Rumble extends SubsystemBase {

	private static Rumble m_instance = null;
	private final XboxController[] m_controllers;
	// private XboxController controller;
	// private boolean rumble;

	// FOr creation of the rumble object
	private Rumble(XboxController... controllers) {
		m_controllers = controllers;
	}

	public static Rumble getInstance() {
		return m_instance;
	}

	//Call this in robot Container first  or you will get a NULL POINTER
	public static Rumble getInstance(XboxController... controllers) {
		if (m_instance == null) {
			m_instance = new Rumble(controllers);

		}
		return m_instance;
	}

	/**
	 * @param controllerIdx
	 * @param rumbleStrength 0 for off, 1 for weak, 2 for medium, 3 for strong
	 * @param duration       the duration in seconds of the rumble. -1 will be an indefinite rumble
	 * @return returns RunCommand MUST call .schedule for it to take effect.
	 */
	public CommandGroupBase runRumble(int controllerIdx, int rumbleStrength, double duration) {
		double[] strength = new double[2]; // [left, right]
		switch (rumbleStrength) {
			case 1:
				strength[0] = 1d;
				strength[1] = 0d;
				break;
			case 2:
				strength[0] = 0d;
				strength[1] = 1d;
				break;
			case 3:
				strength[0] = 1d;
				strength[1] = 1d;
				break;
			default:
				strength[0] = 0d;
				strength[1] = 0d;
				break;
		}
		return new RunCommand(() -> {
			m_controllers[controllerIdx].setRumble(GenericHID.RumbleType.kLeftRumble, strength[0]);
			m_controllers[controllerIdx].setRumble(GenericHID.RumbleType.kRightRumble, strength[1]);
		}, this).withTimeout(duration).andThen(
				new InstantCommand(() -> {
					m_controllers[controllerIdx].setRumble(GenericHID.RumbleType.kLeftRumble, 0);
					m_controllers[controllerIdx].setRumble(GenericHID.RumbleType.kRightRumble, 0);
				}, this));
	}

	private class RumbleCommand extends CommandBase {

	}
}
