package frc.robot.subsystems.drivetrain.commands.auto_routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.commands.PathResetOdometry;
import frc.robot.subsystems.drivetrain.commands.TrajectoryFollow;

public class FenderShot extends SequentialCommandGroup {

	public FenderShot() {
		super(
				// new WaitCommand(2),  // fender_aim - return a speed in hood state
				// new Shoot().withTimeout(2),
				new PathResetOdometry("simple"),
				new TrajectoryFollow("simple"));
	}

}