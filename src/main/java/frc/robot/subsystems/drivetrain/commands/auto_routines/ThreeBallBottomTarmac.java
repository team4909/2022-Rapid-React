package frc.robot.subsystems.drivetrain.commands.auto_routines;

import java.time.Instant;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.commands.PathResetOdometry;
import frc.robot.subsystems.drivetrain.commands.TrajectoryFollow;
import frc.robot.subsystems.intake.IntakeFeeder;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.ShootCmd;
import frc.robot.subsystems.vision.VisionSubsystem;

public class ThreeBallBottomTarmac extends SequentialCommandGroup {

    //TODO maybe these shouldnt be used statically maybe instead make them requirements and pass them in idk really
    IntakeFeeder intake_ = IntakeFeeder.getInstance();
    Shooter shooter_ = Shooter.getInstance();
    VisionSubsystem vision_ = VisionSubsystem.getInstance();

    public ThreeBallBottomTarmac(SubsystemBase... requirements) {
        addCommands( 

        new PathResetOdometry("Tarmac-A"), (
            new TrajectoryFollow("Tarmac-A").withTimeout(2)
            .raceWith(new RunCommand(intake_::intake, intake_))
        )
        .andThen(new InstantCommand(intake_::stopIntake)),

        new RunCommand(vision_::setLimelightOffset).withTimeout(2)
        .andThen(() -> vision_.setLimelightOffset(0)),

        new InstantCommand(
            () -> shooter_.setVelocityGoal(Constants.kFenderShotVelocity, false))
            .perpetually()
            .withInterrupt(() -> shooter_.spunUp()
        ),
        new RunCommand(intake_::shoot).withTimeout(3)
        .andThen(new InstantCommand(intake_::stopIntake))
        .andThen(new InstantCommand(shooter_::stop)),
        
       (new TrajectoryFollow("A-B").withTimeout(3)
        .raceWith(new RunCommand(intake_::intake, intake_)))
        .andThen(new InstantCommand(intake_::stopIntake)),

        new InstantCommand(
            () -> shooter_.setVelocityGoal(Constants.kFenderShotVelocity, false))
            .perpetually()
            .withInterrupt(() -> shooter_.spunUp()
        ),
        new RunCommand(intake_::shoot).withTimeout(2)
        .andThen(new InstantCommand(intake_::stopIntake))
        .andThen(new InstantCommand(shooter_::stop))
        );
      
        addRequirements(requirements);
    }
    
}