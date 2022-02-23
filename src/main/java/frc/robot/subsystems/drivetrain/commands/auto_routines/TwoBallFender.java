package frc.robot.subsystems.drivetrain.commands.auto_routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drivetrain.commands.PathResetOdometry;
import frc.robot.subsystems.drivetrain.commands.TrajectoryFollow;
import frc.robot.subsystems.shooter.commands.Shoot;

public class TwoBallFender extends SequentialCommandGroup{

    public TwoBallFender(){
        super(            

            new PathResetOdometry("Tarmac-A"),
            new TrajectoryFollow("Tarmac-A"));
            
    }
   
}