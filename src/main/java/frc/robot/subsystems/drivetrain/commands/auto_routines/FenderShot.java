package frc.robot.subsystems.drivetrain.commands.auto_routines;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drivetrain.commands.TrajectoryFollow;
import frc.robot.subsystems.shooter.commands.Shoot;

public class FenderShot extends SequentialCommandGroup{

    public FenderShot(){
        super(            
            new WaitCommand(2),  // fender_aim - return a speed in hood state
            new WaitCommand(2),  // rev - get us up to speed for fender aim
            new Shoot(),
            new TrajectoryFollow("FenderTaxi"));
    }

    //     addCommands(
    //         new WaitCommand(2),  // fender_aim - return a speed in hood state
    //         new WaitCommand(2),  // rev - get us up to speed for fender aim
    //         new ParallelDeadlineGroup(  
    //             new WaitCommand(3),  // shoot - shoot the ball
    //             new WaitCommand(4)   // rev
    //         ),
    //         new TrajectoryFollow("FenderTaxi"));      
}