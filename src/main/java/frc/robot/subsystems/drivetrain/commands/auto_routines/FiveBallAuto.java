package frc.robot.subsystems.drivetrain.commands.auto_routines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.commands.PathResetOdometry;
import frc.robot.subsystems.drivetrain.commands.TrajectoryFollow;
import frc.robot.subsystems.intake.IntakeFeeder;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.VisionSubsystem;

public class FiveBallAuto extends SequentialCommandGroup {

    IntakeFeeder intake_ = IntakeFeeder.getInstance();
    Shooter shooter_ = Shooter.getInstance();
    VisionSubsystem vision_ = VisionSubsystem.getInstance();
    Hood hood_ = Hood.getInstance();

    public FiveBallAuto() {
        addCommands( 
        shooter_.setGoalDemand(4711),
        new PathResetOdometry("Tarmac-Almost-A"), 
        (
        new TrajectoryFollow("Tarmac-Almost-A").withTimeout(2)
        .raceWith(new RunCommand(intake_::intake, intake_))
        )
        .andThen(vision_.LimelightAim())
        .andThen(new InstantCommand(intake_::stopIntake))
        .andThen(() -> hood_.setHoodAngle(29.5)),

        new RunCommand(intake_::shoot).withTimeout(3),

       (new TrajectoryFollow("Near-A-B").withTimeout(2)
        .raceWith(new RunCommand(intake_::intake, intake_)))
        .andThen(new InstantCommand(intake_::stopIntake))
        .andThen(vision_.LimelightAim())
        .andThen(() -> hood_.setHoodAngle(22)),
        

        new RunCommand(intake_::shoot).withTimeout(3)
        .andThen(new InstantCommand(intake_::stopIntake))
        .andThen(new InstantCommand(shooter_::stop)),

        new TrajectoryFollow("B-CD").withTimeout(2.3)
            .raceWith(new RunCommand(intake_::intake, intake_))
            .andThen(shooter_.setGoalDemand(4711)),

        new TrajectoryFollow("B-CD-Reverse").withTimeout(2.3),
        // shooter_.runShooter(3553)
        shooter_.setGoalDemand(5600)
        // new InstantCommand(() -> shooter_.setVelocityGoal(3553))
        .andThen(vision_.LimelightAim())
        .andThen(() -> hood_.setHoodAngle(29.5)),
        new RunCommand(intake_::shoot).withTimeout(3)
            .andThen(new InstantCommand(intake_::stopIntake))
            .andThen(new InstantCommand(shooter_::stop))
       
        );
        
   }
    
}