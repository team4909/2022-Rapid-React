package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Shoot extends ParallelDeadlineGroup { // FixMe: delete
	public Shoot() {
		super(
				new WaitCommand(1), // feed
				new WaitCommand(1)); // rev
	}
}