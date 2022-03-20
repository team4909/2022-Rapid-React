package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * Class that makes it so that all types of controllers have the same buttons/controls and it's the same layout.
 */
public class VictorController extends Joystick {
	private static final String F310_NAME = "Controller (Gamepad F310)", DUALSHOCK_NAME = "Wireless Controller", XBOX_NAME = "Controller (Xbox One For Windows)";
	private final SendableChooser<ControllerType> chooser = new SendableChooser<>();
	private final int port;
	volatile ControllerType type;
	private double leftTriggerDeadband = 0, rightTriggerDeadband = 0, leftTriggerThreshold = 0.5, rightTriggerThreshold = 0.5;
	private double leftStickXDeaband = 0, rightStickXDeadband = 0, leftStickYDeadband = 0, rightStickYDeadband = 0;

	/**
	 * Construct an instance of a joystick. The joystick index is the USB port on the drivers
	 * station.
	 *
	 * @param port The port on the Driver Station that the joystick is plugged into.
	 */
	public VictorController(int port) {
		super(port);
		this.port = port;
		chooser.addOption("Auto", ControllerType.USE_NAME);
		chooser.addOption("Logitech F310", ControllerType.LOGITECH);
		chooser.addOption("Dualshock 4", ControllerType.PS4);
		chooser.addOption("Xbox Controller", ControllerType.XBOX);
		updateControllerType();
	}

	/**
	 * DISCLAIMER: At the start of every run mode controllers should be set
	 * Sets the controllers type based on the name that is given by the DriverStation
	 */
	public void updateControllerType() {
		/*if(usingShuffleBoardToSet())
			this.type = chooser.getSelected();
		else*/
		this.type = ControllerType.getTypeFromName(getName());
		SmartDashboard.putString((port == 0 ? "Driver" : "Operator") + " Controller Type", this.type.name());
	}

	/**
	 * Sets the left stick X deadband
	 *
	 * @param deadband Deadband to set
	 * @return Controller
	 */
	public VictorController setLeftStickXDeadband(double deadband) {
		this.leftStickXDeaband = deadband;
		return this;
	}


	/**
	 * Sets the right stick X deadband
	 *
	 * @param deadband Deadband to set
	 * @return Controller
	 */
	public VictorController setRightStickXDeadband(double deadband) {
		this.rightStickXDeadband = deadband;
		return this;
	}

	/**
	 * Sets the left stick Y deadband
	 *
	 * @param leftStickYDeadband Deadband to set
	 * @return Controller
	 */
	public VictorController setLeftStickYDeadband(double leftStickYDeadband) {
		this.leftStickYDeadband = leftStickYDeadband;
		return this;
	}

	/**
	 * Sets the right stick Y deadband
	 *
	 * @param rightStickYDeadband Deadband to set
	 * @return Controller
	 */
	public VictorController setRightStickYDeadband(double rightStickYDeadband) {
		this.rightStickYDeadband = rightStickYDeadband;
		return this;
	}

	public VictorController setLeftTriggerButtonThreshold(double threshold) {
		this.leftTriggerThreshold = threshold;
		return this;
	}

	public VictorController setRightTriggerButtonThreshold(double threshold) {
		this.rightTriggerThreshold = threshold;
		return this;
	}

	/**
	 * Gets the left and right trigger sum for driving. (RT - LT)
	 *
	 * @return Trigger sum
	 */
	public double getTriggerSum() {
		return getRightTrigger() - getLeftTrigger();
	}

	public void setTriggerDeadbands(double leftTriggerDeadband, double rightTriggerDeadband) {
		setLeftTriggerDeadband(leftTriggerDeadband);
		setRightTriggerDeadband(rightTriggerDeadband);
	}

	/**
	 * Gets the left joystick X value
	 *
	 * @return Left joystick X value
	 */
	public double getLeftStickX() {
		double stickValue = getRawAxis(type.constants.leftX());
		return Math.abs(stickValue) > leftStickXDeaband ? stickValue : 0;
	}

	/**
	 * Gets the left joystick Y value
	 *
	 * @return Left joystick Y value
	 */
	public double getLeftStickY() {
		double stickValue = getRawAxis(type.constants.leftY());
		return Math.abs(stickValue) > leftStickYDeadband ? stickValue : 0;
	}

	/**
	 * Gets the right joystick X value
	 *
	 * @return Right joystick X value
	 */
	public double getRightStickX() {
		double stickValue = getRawAxis(type.constants.rightX());
		return Math.abs(stickValue) > rightStickXDeadband ? stickValue : 0;
	}

	/**
	 * Gets the Right joystick Y value
	 *
	 * @return Right joystick Y value
	 */
	public double getRightStickY() {
		double stickValue = getRawAxis(type.constants.rightY());
		return Math.abs(stickValue) > rightStickYDeadband ? stickValue : 0;
	}

	/**
	 * Gets the left stick angle
	 *
	 * @return Left joystick angle
	 */
	public double getLeftStickDirection() {
		return Math.toDegrees(Math.atan2(getLeftStickX(), getLeftStickY()));
	}

	public double getLeftStickDirection(double lowestMagnitude) {
		return getLeftStickMagnitude() < lowestMagnitude ? 0 : getLeftStickDirection();
	}

	/**
	 * Gets the right stick angle
	 *
	 * @return Right joystick angle
	 */
	public double getRightStickDirection() {
		return Math.toDegrees(Math.atan2(getRightStickX(), getRightStickY()));
	}

	public double getRightStickDirection(double lowestMagnitude) {
		return getRightStickMagnitude() < lowestMagnitude ? 0 : getRightStickDirection();
	}

	/**
	 * Gets the left stick magnitude
	 *
	 * @return Left stick magnitude
	 */
	public double getLeftStickMagnitude() {
		return Math.hypot(getLeftStickX(), getLeftStickY());
	}

	/**
	 * Gets the right stick magnitude
	 *
	 * @return Right stick magnitude
	 */
	public double getRightStickMagnitude() {
		return Math.hypot(getRightStickX(), getRightStickY());
	}

	/**
	 * Gets left stick press. It's the click when you press the joystick in
	 *
	 * @return Left stick press value
	 */
	public boolean getLeftStickPress() {
		return super.getRawButton(type.constants.l3());
	}

	public boolean getLeftStickPressPressed() {
		return super.getRawButtonPressed(type.constants.l3());
	}

	public boolean getLeftStickPressReleased() {
		return super.getRawButtonReleased(type.constants.l3());
	}

	public JoystickButton getLeftStickJoystickButton() {
		return new SafeJButton(this, type.constants.l3());
	}

	/**
	 * Gets right stick press. It's the click when you press the joystick in
	 *
	 * @return Right stick press value
	 */
	public boolean getRightStickPress() {
		return super.getRawButton(type.constants.r3());
	}

	public boolean getRightStickPressPressed() {
		return super.getRawButtonPressed(type.constants.r3());
	}

	public boolean getRightStickPressReleased() {
		return super.getRawButtonReleased(type.constants.r3());
	}

	public JoystickButton getRightStickJoystickButton() {
		return new SafeJButton(this, type.constants.r3());
	}

	/**
	 * Gets the left bumper
	 *
	 * @return Left bumper value
	 */
	public boolean getLeftBumper() {
		return super.getRawButton(type.constants.lb());
	}

	public boolean getLeftBumperPressed() {
		return super.getRawButtonPressed(type.constants.lb());
	}

	public boolean getLeftBumperReleased() {
		return super.getRawButtonReleased(type.constants.lb());
	}

	public JoystickButton getLeftBumperJoystickButton() {
		return new SafeJButton(this, type.constants.lb());
	}

	/**
	 * Gets the right bumper
	 *
	 * @return Right bumper value
	 */
	public boolean getRightBumper() {
		return super.getRawButton(type.constants.rb());
	}

	public boolean getRightBumperPressed() {
		return super.getRawButtonPressed(type.constants.rb());
	}

	public boolean getRightBumperReleased() {
		return super.getRawButtonReleased(type.constants.rb());
	}

	public JoystickButton getRightBumperJoystickButton() {
		return new SafeJButton(this, type.constants.rb());
	}

	/**
	 * Gets the left trigger
	 *
	 * @return Left trigger value
	 */
	public double getLeftTrigger() {
		double triggerValue = getRawAxis(type.constants.leftTrigger());
		triggerValue = type == ControllerType.PS4 ? (triggerValue + 1) / 2 : triggerValue;
		return triggerValue > leftTriggerDeadband ? triggerValue : 0;
	}

	public boolean getLeftTriggerBoolean() {
		return getLeftTrigger() >= this.leftTriggerThreshold;
	}

	public JoystickButton getLeftTriggerJoystickButton() {
		return new SafeJButton(this, -1) {
			@Override
			public boolean get() {
				return VictorController.this.getLeftTriggerBoolean();
			}
		};
	}

	/**
	 * Sets the deadband for the left trigger
	 *
	 * @param leftTriggerDeadband deadband
	 */
	public void setLeftTriggerDeadband(double leftTriggerDeadband) {
		this.leftTriggerDeadband = leftTriggerDeadband;
	}

	/**
	 * Gets the right trigger
	 *
	 * @return Right trigger
	 */
	public double getRightTrigger() {
		double triggerValue = getRawAxis(type.constants.rightTrigger());
		triggerValue = type == ControllerType.PS4 ? (triggerValue + 1) / 2 : triggerValue;
		return triggerValue > rightTriggerDeadband ? triggerValue : 0;
	}

	public boolean getRightTriggerBoolean() {
		return getRightTrigger() >= this.rightTriggerThreshold;
	}

	public JoystickButton getRightTriggerJoystickButton() {
		return new SafeJButton(this, -1) {
			@Override
			public boolean get() {
				return VictorController.this.getRightTriggerBoolean();
			}
		};
	}

	/**
	 * Sets the deadband for the right trigger
	 *
	 * @param rightTriggerDeadband deadband
	 */
	public void setRightTriggerDeadband(double rightTriggerDeadband) {
		this.rightTriggerDeadband = rightTriggerDeadband;
	}

	/**
	 * Returns the bottom button on the right side of the controller
	 *
	 * @return Bottom button
	 */
	public boolean a() {
		return super.getRawButton(type.constants.a());
	}

	public boolean aPressed() {
		return super.getRawButtonPressed(type.constants.a());
	}

	public boolean aReleased() {
		return super.getRawButtonReleased(type.constants.a());
	}

	public JoystickButton getAJoystickButton() {
		return new SafeJButton(this, type.constants.a());
	}

	/**
	 * Returns the right button on the right side of the controller
	 *
	 * @return Right button
	 */
	public boolean b() {
		return super.getRawButton(type.constants.b());
	}

	public boolean bPressed() {
		return super.getRawButtonPressed(type.constants.b());
	}

	public boolean bReleased() {
		return super.getRawButtonReleased(type.constants.b());
	}

	public JoystickButton getBJoystickButton() {
		return new SafeJButton(this, type.constants.b());
	}

	/**
	 * Returns the left button on the right side of the controller
	 *
	 * @return Left button
	 */
	public boolean x() {
		return super.getRawButton(type.constants.x());
	}

	public boolean xPressed() {
		return super.getRawButtonPressed(type.constants.x());
	}

	public boolean xReleased() {
		return super.getRawButtonReleased(type.constants.x());
	}

	public JoystickButton getXJoystickButton() {
		return new SafeJButton(this, type.constants.x());
	}

	/**
	 * Returns the top button on the right side of the controller
	 *
	 * @return Top button
	 */
	public boolean y() {
		return super.getRawButton(type.constants.y());
	}

	public boolean yPressed() {
		return super.getRawButtonPressed(type.constants.y());
	}

	public boolean yReleased() {
		return super.getRawButtonReleased(type.constants.y());
	}

	public JoystickButton getYJoystickButton() {
		return new SafeJButton(this, type.constants.y());
	}

	/**
	 * Returns the start button.
	 * On PS4 this button is called OPTIONS
	 *
	 * @return Start button
	 */
	public boolean getStart() {
		return super.getRawButton(type.constants.start());
	}

	public boolean getStartPressed() {
		return super.getRawButtonPressed(type.constants.start());
	}

	public boolean getStartReleased() {
		return super.getRawButtonReleased(type.constants.start());
	}

	public JoystickButton getStartJoystickButton() {
		return new SafeJButton(this, type.constants.start());
	}

	/**
	 * Returns the back button
	 *
	 * @return Back button
	 */
	public boolean getBack() {
		return super.getRawButton(type.constants.back());
	}

	public boolean getBackPressed() {
		return super.getRawButtonPressed(type.constants.back());
	}

	public boolean getBackReleased() {
		return super.getRawButtonReleased(type.constants.back());
	}

	public JoystickButton getBackJoystickButton() {
		return new SafeJButton(this, type.constants.back());
	}

	/**
	 * Gets the dpad up value
	 *
	 * @return DPad up value
	 */
	public boolean getDPadUp() {
		int dPadValue = getPOV();
		var direction = 0;
		return (dPadValue == direction) || (dPadValue == (direction + 45) % 360) || (dPadValue == (direction + 315) % 360);
	}

	public JoystickButton getDPadUpJoystickButton() {
		return new SafeJButton(this, -1) {
			@Override
			public boolean get() {
				return VictorController.this.getDPadUp();
			}
		};
	}

	/**
	 * Gets the dpad down value
	 *
	 * @return DPad down value
	 */
	public boolean getDPadDown() {
		int dPadValue = getPOV();
		var direction = 180;
		return (dPadValue == direction) || (dPadValue == (direction + 45) % 360) || (dPadValue == (direction + 315) % 360);
	}

	public JoystickButton getDPadDownJoystickButton() {
		return new SafeJButton(this, -1) {
			@Override
			public boolean get() {
				return VictorController.this.getDPadDown();
			}
		};
	}

	/**
	 * Gets the dpad left value
	 *
	 * @return DPad left value
	 */
	public boolean getDPadLeft() {
		int dPadValue = getPOV();
		var direction = 270;
		return (dPadValue == direction) || (dPadValue == (direction + 45) % 360) || (dPadValue == (direction + 315) % 360);
	}

	public JoystickButton getDPadLeftJoystickButton() {
		return new SafeJButton(this, -1) {
			@Override
			public boolean get() {
				return VictorController.this.getDPadLeft();
			}
		};
	}

	/**
	 * Gets the dpad right value
	 *
	 * @return DPad right value
	 */
	public boolean getDPadRight() {
		int dPadValue = getPOV();
		var direction = 90;
		return (dPadValue == direction) || (dPadValue == (direction + 45) % 360) || (dPadValue == (direction + 315) % 360);
	}

	public JoystickButton getDPadRightJoystickButton() {
		return new SafeJButton(this, -1) {
			@Override
			public boolean get() {
				return VictorController.this.getDPadRight();
			}
		};
	}

	/**
	 * Rumbles both hands at a given percentage
	 *
	 * @param value Sets rumble percent [0, 1]
	 */
	public void rumble(double value) {
		rumbleLeft(value);
		rumbleRight(value);
	}

	/**
	 * Rumbles left hand at a given percentage
	 *
	 * @param value Sets rumble perce:wq
	 *              <p>
	 *              :ew   nt [0, 1]
	 */
	public void rumbleLeft(double value) {
		setRumble(RumbleType.kLeftRumble, value);
	}

	/**
	 * Rumbles right hand at a given percentage
	 *
	 * @param value Sets rumble percent [0, 1]
	 */
	public void rumbleRight(double value) {
		setRumble(RumbleType.kRightRumble, value);
	}

	public int getStartButtonNumber() {
		return type.constants.start();
	}

	public int getLeftBumperNumber() {
		return type.constants.lb();
	}

	public int getRightBumperNumber() {
		return type.constants.rb();
	}

	public int getXButtonNumber() {
		return type.constants.x();
	}

	public int getBButtonNumber() {
		return type.constants.b();
	}

	public int getAButtonNumber() {
		return type.constants.a();
	}

	public int getYButtonNumber() {
		return type.constants.y();
	}


	/**
	 * Sends all necessary values to NetworkTables
	 */
	public void updateShuffleboard() {
		String driverOperator = port == 0 ? "Driver" : "Operator";
		try {
			try {
				Shuffleboard.getTab("Controllers").add(driverOperator + "Controller Type", this.chooser);
			} catch (IllegalArgumentException ignored) {

			}
			SmartDashboard.putString(driverOperator + " Controller Type", this.type.name());
			SmartDashboard.putBoolean(driverOperator + " A", a());
			SmartDashboard.putBoolean(driverOperator + " B", b());
			SmartDashboard.putBoolean(driverOperator + " X", x());
			SmartDashboard.putBoolean(driverOperator + " Y", y());
			SmartDashboard.putBoolean(driverOperator + " L3", getLeftStickPress());
			SmartDashboard.putNumber(driverOperator + " Right Y", getRightStickY());
			SmartDashboard.putNumber(driverOperator + " Right Trigger", getRightTrigger());
			SmartDashboard.putBoolean(driverOperator + " DPad Up", getDPadUp());
			SmartDashboard.putNumber(driverOperator + " Left X", getLeftStickX());
			SmartDashboard.putNumber(driverOperator + " Left Y", getLeftStickY());

		} catch (NullPointerException npe) {
			SmartDashboard.putString("Error", "Can't update dash");
		}
	}

	private enum ControllerType {
		LOGITECH("Gamepad F310", new Constants()),
		PS4("Wireless Controller", new PS4Constants()),
		XBOX("Xbox One For Windows", new XBoxConstants()),
		USE_NAME("Auto", null);

		final String displayName;
		private final Constants constants;


		ControllerType(String displayName, Constants constants) {
			this.displayName = displayName;
			this.constants = constants;
		}

		@SuppressWarnings("IfCanBeSwitch")
		static ControllerType getTypeFromName(String name) {
			if (name.equals(F310_NAME)) return ControllerType.LOGITECH;
			else if (name.equals(DUALSHOCK_NAME)) return ControllerType.PS4;
			else if (name.equals(XBOX_NAME)) return ControllerType.XBOX;
			else return ControllerType.LOGITECH;
		}
	}

	/**
	 * The constants contain methods and not variables to make the inheritance work
	 */
	private static class Constants {
		int a() {
			return 1;
		}

		int b() {
			return 2;
		}

		int x() {
			return 3;
		}

		int y() {
			return 4;
		}

		int lb() {
			return 5;
		}

		int rb() {
			return 6;
		}

		int back() {
			return 7;
		}

		int start() {
			return 8;
		}

		int l3() {
			return 9;
		}

		int r3() {
			return 10;
		}

		int leftTrigger() {
			return 2;
		}

		int rightTrigger() {
			return 3;
		}

		int leftX() {
			return 0;
		}

		int leftY() {
			return 1;
		}

		int rightX() {
			return 4;
		}

		int rightY() {
			return 5;
		}

		//Don't ask
		int getButtonIDFromID(int id) {
			switch (id) {
				case 1:
					return a();
				case 2:
					return b();
				case 3:
					return x();
				case 4:
					return y();
				case 5:
					return lb();
				case 6:
					return rb();
				case 7:
					return back();
				case 8:
					return start();
				case 9:
					return l3();
				case 10:
					return r3();
				default:
					return 0;
			}
		}
	}

	//If these constants are wrong then decrement all the buttons
	private static class PS4Constants extends Constants {
		@Override
		int a() {
			return 2;
		}

		@Override
		int b() {
			return 3;
		}

		@Override
		int x() {
			return 1;
		}

		@Override
		int y() {
			return 4;
		}

		@Override
		int lb() {
			return 5;
		}

		@Override
		int rb() {
			return 6;
		}

		@Override
		int back() {
			return 9;
		}

		@Override
		int start() {
			return 10;
		}

		@Override
		int l3() {
			return 11;
		}

		@Override
		int r3() {
			return 12;
		}

		@Override
		int leftTrigger() {
			return 3;
		}

		@Override
		int rightTrigger() {
			return 4;
		}

		@Override
		int leftX() {
			return 0;
		}

		@Override
		int leftY() {
			return 1;
		}

		@Override
		int rightX() {
			return 2;
		}

		@Override
		int rightY() {
			return 5;
		}

		@Override
		int getButtonIDFromID(int id) {
			switch (id) {
				case 1:
					return a();
				case 2:
					return b();
				case 3:
					return x();
				case 4:
					return y();
				case 5:
					return lb();
				case 6:
					return rb();
				case 7:
					return back();
				case 8:
					return start();
				case 9:
					return l3();
				case 10:
					return r3();
				default:
					return 0;
			}
		}


/* Dualshock button map
		//Buttons
		public int A = 2;
		public int B = 3;
		public int X = 1;
		public int Y = 4;
		public int LB = 5;
		public int RB = 6;
		public int BACK = 9;
		public int START = 10;
		public int L3 = 11;
		public int R3 = 12;

		//Analog
		public int LEFT_TRIGGER = 3;
		public int RIGHT_TRIGGER = 4;
		public int LEFT_X = 0;
		public int LEFT_Y = 1;
		public int RIGHT_X = 2;
		public int RIGHT_Y = 5;
*/
	}

	private static class XBoxConstants extends Constants {
		int a() {
			return 1;
		}

		int b() {
			return 2;
		}

		int x() {
			return 3;
		}

		int y() {
			return 4;
		}

		int lb() {
			return 5;
		}

		int rb() {
			return 6;
		}

		int back() {
			return 7;
		}

		int start() {
			return 8;
		}

		int l3() {
			return 9;
		}

		int r3() {
			return 10;
		}

		int leftTrigger() {
			return 2;
		}

		int rightTrigger() {
			return 3;
		}

		int leftX() {
			return 0;
		}

		int leftY() {
			return 1;
		}

		int rightX() {
			return 4;
		}

		int rightY() {
			return 5;
		}
	}

	private class SafeJButton extends JoystickButton {
		private final int buttonNumber;
		private boolean
				whenPressedAssigned = false,
				whileHeldAssigned = false,
				whenHeldAssigned = false,
				whenReleasedAssigned = false,
				toggleWhenPressedAssigned = false,
				cancelWhenPressedAssigned = false;

		/**
		 * Creates a joystick button for triggering commands.
		 *
		 * @param joystick     The GenericHID object that has the button (e.g. Joystick, KinectStick, etc)
		 * @param buttonNumber The button number (see {@link GenericHID#getRawButton(int) }
		 */
		public SafeJButton(GenericHID joystick, int buttonNumber) {
			super(joystick, buttonNumber);
			this.buttonNumber = buttonNumber;
		}

		@Override
		public Button whenPressed(Command command, boolean interruptible) {
			// if (!whenPressedAssigned) {
			// 	whenPressedAssigned = true;
			// } else {
			// 	throw new ButtonReassignedException("whenPressed");
			// }
			return super.whenPressed(command, interruptible);
		}

		@Override
		public Button whenPressed(Command command) {
			return whenPressed(command, true);
		}

		@Override
		public Button whenPressed(Runnable toRun, Subsystem... requirements) {
			if (!whenPressedAssigned) {
				whenPressedAssigned = true;
			} else {
				throw new ButtonReassignedException("whenPressed");
			}
			return super.whenPressed(toRun, requirements);
		}

		@Override
		public Button whileHeld(Command command, boolean interruptible) {
			if (!whileHeldAssigned) {
				whileHeldAssigned = true;
			} else {
				throw new ButtonReassignedException("whileHeld");
			}
			return super.whileHeld(command, interruptible);
		}

		@Override
		public Button whileHeld(Command command) {
			return whileHeld(command, true);
		}

		@Override
		public Button whileHeld(Runnable toRun, Subsystem... requirements) {
			if (!whileHeldAssigned) {
				whileHeldAssigned = true;
			} else {
				throw new ButtonReassignedException("whileHeld");
			}
			return super.whileHeld(toRun, requirements);
		}

		@Override
		public Button whenHeld(Command command, boolean interruptible) {
			if (!whenHeldAssigned) {
				whenHeldAssigned = true;
			} else {
				throw new ButtonReassignedException("whenHeld");
			}
			return super.whenHeld(command, interruptible);
		}

		@Override
		public Button whenHeld(Command command) {
			return whenHeld(command, true);
		}

		@Override
		public Button whenReleased(Command command, boolean interruptible) {
			if (!whenReleasedAssigned) {
				whenReleasedAssigned = true;
			} else {
				throw new ButtonReassignedException("whenReleased");
			}
			return super.whenReleased(command, interruptible);
		}

		@Override
		public Button whenReleased(Command command) {
			return whenReleased(command, true);
		}

		@Override
		public Button whenReleased(Runnable toRun, Subsystem... requirements) {
			if (!whenReleasedAssigned) {
				whenReleasedAssigned = true;
			} else {
				throw new ButtonReassignedException("whenReleased");
			}
			return super.whenReleased(toRun, requirements);
		}

		@Override
		public Button toggleWhenPressed(Command command, boolean interruptible) {
			if (!toggleWhenPressedAssigned) {
				toggleWhenPressedAssigned = true;
			} else {
				throw new ButtonReassignedException("toggleWhenPressed");
			}
			return super.toggleWhenPressed(command, interruptible);
		}

		@Override
		public Button toggleWhenPressed(Command command) {
			return toggleWhenPressed(command, true);
		}

		@Override
		public Button cancelWhenPressed(Command command) {
			if (!cancelWhenPressedAssigned) {
				cancelWhenPressedAssigned = true;
			} else {
				throw new ButtonReassignedException("cancelWhenPressed");
			}
			return super.cancelWhenPressed(command);
		}

		public class ButtonReassignedException extends RuntimeException {
			private ButtonReassignedException(String actionName) {
				super(String.format("%s has already been assigned for button number %d on %s controller at port %d.",
						actionName, SafeJButton.this.buttonNumber, VictorController.this.type.displayName, VictorController.this.port));
			}
		}
	}

}
