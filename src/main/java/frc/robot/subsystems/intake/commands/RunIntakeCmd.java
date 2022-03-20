package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.IntakeFeeder;

public class RunIntakeCmd extends CommandBase {
	private final IntakeFeeder intakeFeeder_;

	public RunIntakeCmd() {
		intakeFeeder_ = IntakeFeeder.getInstance();
	}

	@Override
	public void initialize() {
		intakeFeeder_.intake();

	}

	@Override
	public void end(boolean interrupted) {
		intakeFeeder_.stopIntake();

	}
}
