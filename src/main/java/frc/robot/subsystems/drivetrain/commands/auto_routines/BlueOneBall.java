package frc.robot.subsystems.drivetrain.commands.auto_routines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.commands.PathResetOdometry;
import frc.robot.subsystems.drivetrain.commands.TrajectoryFollow;
import frc.robot.subsystems.intake.IntakeFeeder;
import frc.robot.subsystems.shooter.Shooter;

public class BlueOneBall extends SequentialCommandGroup{

    private final Shooter shooter_ = Shooter.getInstance();
    private final IntakeFeeder intake_ = IntakeFeeder.getInstance();

    public BlueOneBall() {
        
        addCommands(
            shooter_.setGoalCommand(Constants.kFenderShotVelocity),
            
            new RunCommand(intake_::shoot).withTimeout(3)
            .andThen(new InstantCommand(intake_::stopIntake))
            .andThen(new InstantCommand(shooter_::stop)),
    

        new PathResetOdometry("Tarmac-Almost-A"),
        new TrajectoryFollow("Tarmac-Almost-A").withTimeout(2)
        );
        
    }
}
