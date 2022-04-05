package frc.robot.subsystems.drivetrain.commands.auto_routines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.commands.PathResetOdometry;
import frc.robot.subsystems.drivetrain.commands.TrajectoryFollow;
import frc.robot.subsystems.intake.IntakeFeeder;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.AutoShot;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionSubsystem;

public class TwoBallHanger extends SequentialCommandGroup {

    IntakeFeeder intake_ = IntakeFeeder.getInstance();
    Shooter shooter_ = Shooter.getInstance();
    Vision vision_ = Vision.getInstance();
    Hood hood_ = Hood.getInstance();

    public TwoBallHanger(double offset) {
        addCommands(
            new RunCommand(intake_::intake, intake_).withTimeout(1.5),
            new InstantCommand(intake_::resetBallCount),
            new PathResetOdometry("TarmacN-E"), 
            new TrajectoryFollow("TarmacN-E").withTimeout(1.7),

            new InstantCommand(intake_::stopIntake)
            .andThen(new AutoShot(vision_, shooter_, hood_).withTimeout(2)),
        

        new RunCommand(intake_::shoot).withTimeout(3)
        .andThen(new InstantCommand(intake_::stopIntake))
        .andThen(new InstantCommand(shooter_::stop))
                
        );
    }
    
}
