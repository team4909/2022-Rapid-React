package frc.robot.subsystems.drivetrain.commands.auto_routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.drivetrain.commands.TrajectoryFollow;
import frc.robot.subsystems.shooter.commands.Shoot;

public class NearHangerShot extends SequentialCommandGroup {

    public NearHangerShot() {
        addCommands(
            new TrajectoryFollow("Tarmac-E"),
            new WaitCommand(2), // intake - intake ball E
            new WaitCommand(2), //aim
            new Shoot() //shoot

            );
        
    }
}
