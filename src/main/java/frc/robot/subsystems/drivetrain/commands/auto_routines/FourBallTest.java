package frc.robot.subsystems.drivetrain.commands.auto_routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drivetrain.commands.PathResetOdometry;
import frc.robot.subsystems.drivetrain.commands.TrajectoryFollow;
import frc.robot.subsystems.shooter.commands.Shoot;

public class FourBallTest extends SequentialCommandGroup {

    public FourBallTest() {      
        super(
            new PathResetOdometry("RightAngle1"),
            new TrajectoryFollow("RightAngle1").withTimeout(1.5),
            // new WaitCommand(1),
            // new Shoot().withTimeout(2),
            new TrajectoryFollow("RightAngle2").withTimeout(2.9)
        );
    }

}