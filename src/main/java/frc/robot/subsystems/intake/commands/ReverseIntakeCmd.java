package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.IntakeFeeder;

public class ReverseIntakeCmd extends CommandBase {
	private final IntakeFeeder intakeFeeder_;

	public ReverseIntakeCmd() {
		intakeFeeder_ = IntakeFeeder.getInstance();
	}

	@Override
	public void initialize() {
		// just need to set once, and then it'll stay in that state
		intakeFeeder_.reverseIntake();
	}

	@Override
	public void end(boolean interrupted) {
		intakeFeeder_.stopIntake();
	}
}
