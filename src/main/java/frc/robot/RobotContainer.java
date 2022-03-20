package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Climber.ClimberStates;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.drivetrain.commands.DefaultDriveCommand;
import frc.robot.subsystems.drivetrain.commands.SnapToAngle;
import frc.robot.subsystems.drivetrain.commands.auto_routines.*;
import frc.robot.subsystems.intake.IntakeFeeder;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.io.IOException;

public class RobotContainer {
	private final XboxController m_driverController = new XboxController(0);
	private final XboxController m_operatorController = new XboxController(1);
	private final DrivetrainSubsystem m_drivetrainSubsystem = DrivetrainSubsystem.getInstance();
	private final Climber climber_ = Climber.getInstance();
	private final VisionSubsystem m_VisionSubsystem = VisionSubsystem.getInstance();
	private final PowerDistribution PDH;
	private final Hood m_hoodSubsystem = Hood.getInstance();
	private final Shooter m_shooterSubsystem = Shooter.getInstance();
	private final IntakeFeeder m_intakeSubsystem = IntakeFeeder.getInstance();
	private final SendableChooser<Command> m_chooser = new SendableChooser<>();

	public RobotContainer() {
		Shuffleboard.getTab("Driver");
		PDH = new PowerDistribution(1, ModuleType.kRev);
		PDH.clearStickyFaults();
		LiveWindow.disableTelemetry(PDH);
		System.out.println(PDH.getCurrent(1));

		m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
				m_drivetrainSubsystem,
				() -> -modifyAxis(m_driverController.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * m_drivetrainSubsystem.getPreciseModeScale(),
				() -> -modifyAxis(m_driverController.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * m_drivetrainSubsystem.getPreciseModeScale(),
				() -> ((-modifyAxis(m_driverController.getRightX()) + m_VisionSubsystem.getLimelightOffset()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND) * m_drivetrainSubsystem.getPreciseModeScale()
		));

		configureButtonBindings();
		configureSendableChooser();
	}

	/**
	 * Deadbands a value based on the given constraints
	 *
	 * @param value    The raw value to deadband
	 * @param deadband The deadband constraint
	 * @return Deadbanded value
	 */
	private static double deadband(double value, double deadband) {
		if (Math.abs(value) > deadband) {
			if (value > 0.0) {
				return (value - deadband) / (1.0 - deadband);
			} else {
				return (value + deadband) / (1.0 - deadband);
			}
		} else {
			return 0.0;
		}
	}

	/**
	 * Modifies the value passed in, deadbanded, and squared
	 *
	 * @param value The raw value to be modified
	 * @return The modified value
	 */
	private static double modifyAxis(double value) {
		// Deadband
		value = deadband(value, 0.1);

		// Square the axis
		value = Math.copySign(value * value, value);


		return value;
	}

	private void configureSendableChooser() {
		m_chooser.addOption("Red Three Ball from Bottom of Tarmac", new RedThreeBallBottomTarmac());
		m_chooser.addOption("Two Ball from Bottom of Tarmac", new TwoBallBottomTarmac());
		m_chooser.addOption("Fender Shot", new FenderShot());
		m_chooser.addOption("Two Ball from Top Tarmac", new BlueTwoBallTopTarmac());
		m_chooser.addOption("Blue One Ball Taxi", new BlueOneBall());
		m_chooser.addOption("Blue? FIve Ball Auto", new FiveBallAuto());
		SmartDashboard.putData(m_chooser);
	}

	private void configureButtonBindings() {
		///////////////////////////////
		///      Driver Buttons     ///
		///////////////////////////////

		// Back button zeros the gyroscope
		new Button(m_driverController::getBackButton).whenPressed(m_drivetrainSubsystem::zeroGyroscope);
		new Button(m_driverController::getStartButton)
				.toggleWhenPressed(new InstantCommand(() -> m_VisionSubsystem.switchCamera("Climber Camera"))
						.andThen(new InstantCommand(() -> m_VisionSubsystem.switchCamera("Front Camera"))));

		// Switch Pipelines
		new Button(m_driverController::getRightBumper)

				.whenHeld(new InstantCommand(() -> m_drivetrainSubsystem.setPreciseMode(true)))
				.whenReleased(new InstantCommand(() -> m_drivetrainSubsystem.setPreciseMode(false)));
		new Button(m_driverController::getLeftBumper)
				.whenHeld(new InstantCommand(() -> m_drivetrainSubsystem.setLockInPlace(true)))
				.whenReleased(new InstantCommand(() -> m_drivetrainSubsystem.setLockInPlace(false)));
		new Button(m_driverController::getRightStickButton).whenPressed(m_VisionSubsystem::setPipeline);

		//Fender Shot Angles
		new Button(m_driverController::getAButton).whenPressed(new SnapToAngle(m_driverController, 201, m_drivetrainSubsystem));
		new Button(m_driverController::getBButton).whenPressed(new SnapToAngle(m_driverController, 111, m_drivetrainSubsystem));
		new Button(m_driverController::getXButton).whenPressed(new SnapToAngle(m_driverController, 291, m_drivetrainSubsystem));
		new Button(m_driverController::getYButton).whenPressed(new SnapToAngle(m_driverController, 21, m_drivetrainSubsystem));
		new Button(() -> (m_driverController.getPOV() != -1)).whenPressed(new SnapToAngle(m_driverController, m_drivetrainSubsystem), false);

		// Shoot the shot
		new Trigger(() -> (Math.abs(m_driverController.getRightTriggerAxis()) > 0.7))
				.whenActive(m_intakeSubsystem::shoot)
				.whenInactive(() -> {
					m_intakeSubsystem.stopIntake();
					m_shooterSubsystem.stop();
				});

		new Trigger(() -> (Math.abs(m_driverController.getLeftTriggerAxis())) > 0.7)
				.whenActive(new RunCommand(m_VisionSubsystem::setLimelightOffset)
						.withInterrupt(() -> m_driverController.getLeftTriggerAxis() < 0.7))
				.whenInactive(new InstantCommand(() -> m_VisionSubsystem.setLimelightOffset(0)));

		/////////////////////////////////
		///      Operator Buttons     ///
		/////////////////////////////////
		// Fender shot
		new Button(m_operatorController::getXButton).whenPressed(m_shooterSubsystem.setGoalDemand(Constants.kFenderShotVelocity + 200));
		new Button(m_operatorController::getAButton).whenPressed(m_shooterSubsystem.setGoalDemand(Constants.kFenderShotVelocity)
				.alongWith(new InstantCommand(() -> m_hoodSubsystem.setHoodAngle(13))));
		new Trigger(() -> m_operatorController.getPOV() == 180).whenActive(m_shooterSubsystem.setGoalDemand(Constants.kLongShotVelocity));

		new Trigger(() -> m_operatorController.getPOV() == 90).whenActive(m_shooterSubsystem.setGoalDemand(Constants.kWallShotVelocity));

		// Limelight shot: Stays the same, spins up based on limelight feedback but doesn't shoot
		// Cancel a spin up
		new Button(m_operatorController::getBButton).whenPressed(m_shooterSubsystem::stop);
		new Button(m_operatorController::getYButton).whenPressed(new RunCommand(m_intakeSubsystem::compressBalls).withTimeout(1));

		// Run intake: Operator right trigger
		new Trigger(() -> (Math.abs(m_operatorController.getRightTriggerAxis()) > 0.7))
				.whenActive(m_intakeSubsystem::intake)
				.whenInactive(m_intakeSubsystem::stopIntake);

		// Reverse intake: Operator left trigger
		new Trigger(() -> (Math.abs(m_operatorController.getLeftTriggerAxis()) > 0.7))
				.whenActive(m_intakeSubsystem::reverseIntake)
				.whenInactive(m_intakeSubsystem::stopIntake);

		new Trigger(() -> m_operatorController.getPOV() == 0).whenActive(() -> climber_.setState(ClimberStates.IDLE));
		new Button(m_operatorController::getBackButton).whenPressed(() -> climber_.setState(ClimberStates.CALIBRATE));
		new Button(m_operatorController::getStartButton).whenPressed(() -> climber_.setState(ClimberStates.MID_ALIGN));

		//driver controller
		new Button(m_operatorController::getLeftBumper).whenPressed(() -> climber_.setState(ClimberStates.MID_CLIMB));

		new Button(m_operatorController::getRightBumper).whenPressed(() -> climber_.setState(ClimberStates.HIGHER_CLIMB));
	}

	public void temp() {
		SmartDashboard.putNumber("POV", m_driverController.getPOV());
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 * <p>
	 * %%
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return m_chooser.getSelected();
	}


}
