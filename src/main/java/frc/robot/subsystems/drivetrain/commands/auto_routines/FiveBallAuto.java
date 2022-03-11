package frc.robot.subsystems.drivetrain.commands.auto_routines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.commands.PathResetOdometry;
import frc.robot.subsystems.drivetrain.commands.TrajectoryFollow;
import frc.robot.subsystems.intake.IntakeFeeder;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.LimelightShoot;
import frc.robot.subsystems.vision.VisionSubsystem;

public class FiveBallAuto extends SequentialCommandGroup {

    IntakeFeeder intake_ = IntakeFeeder.getInstance();
    Shooter shooter_ = Shooter.getInstance();
    VisionSubsystem vision_ = VisionSubsystem.getInstance();

    public FiveBallAuto() {
        addCommands( 

            new PathResetOdometry("Tarmac-Almost-A"), 
            (
            new TrajectoryFollow("Tarmac-Almost-A").withTimeout(2)
            .raceWith(new RunCommand(intake_::intake, intake_))
            .alongWith(new LimelightShoot(Constants.kWallShotVelocity, true, true))
            )
            .andThen(new InstantCommand(intake_::stopIntake)),


       (new TrajectoryFollow("Near-A-B").withTimeout(2)
        .raceWith(new RunCommand(intake_::intake, intake_)))
        .alongWith(new InstantCommand(() -> shooter_.setVelocityGoal(Constants.kLongShotVelocity, true)))
        .andThen(new InstantCommand(intake_::stopIntake)),

        new LimelightShoot(Constants.kLongShotVelocity, true, true),

        new TrajectoryFollow("B-CD").withTimeout(2.3)
            .raceWith(new RunCommand(intake_::intake, intake_))
            .raceWith(new InstantCommand(() -> shooter_.setVelocityGoal(Constants.kLongShotVelocity, true))),

        new TrajectoryFollow("B-CD-Reverse").withTimeout(2.3)
            .raceWith(new InstantCommand(() -> shooter_.setVelocityGoal(Constants.kLongShotVelocity, true))),
        new LimelightShoot(Constants.kLongShotVelocity, true, true)
       
        );
        
   }
    
}