package frc.robot.subsystems.drivetrain.commands.auto_routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.drivetrain.commands.PathResetOdometry;
import frc.robot.subsystems.drivetrain.commands.TrajectoryFollow;
import frc.robot.subsystems.shooter.commands.Shoot;

public class FourBallTest extends SequentialCommandGroup {

    public FourBallTest() {      
        super(
            new PathResetOdometry("Tarmac-A"),
            new TrajectoryFollow("Tarmac-A").withTimeout(1.5),
            // new WaitCommand(1),
            // new Shoot().withTimeout(2),
            new TrajectoryFollow("A-C")
        );
    }


    public void initialize() {
        
    }
}
