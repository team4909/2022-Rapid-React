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

public class BlueTwoBallTopTarmac extends SequentialCommandGroup {

    IntakeFeeder intake_ = IntakeFeeder.getInstance();
    Shooter shooter_ = Shooter.getInstance();
    VisionSubsystem vision_ = VisionSubsystem.getInstance();
    Hood hood_ = Hood.getInstance();

    public BlueTwoBallTopTarmac() {
        addCommands(
            shooter_.setGoalDemand(4711),
            new PathResetOdometry("TarmacN-E"), 
            new TrajectoryFollow("TarmacN-E").withTimeout(2)
            .raceWith(new RunCommand(intake_::intake, intake_)),

            new InstantCommand(intake_::stopIntake)
            .andThen(vision_.LimelightAim())
            .andThen(() -> hood_.setHoodAngle(22)),
        

        new RunCommand(intake_::shoot).withTimeout(3)
        .andThen(new InstantCommand(intake_::stopIntake))
        .andThen(new InstantCommand(shooter_::stop))
                
        );
    }
    
}
