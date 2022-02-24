package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeFeeder;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.VisionSubsystem;

public class LimelightShoot extends SequentialCommandGroup{

    private final Shooter shooter_ = Shooter.getInstance();
    private final VisionSubsystem vision_ = VisionSubsystem.getInstance();
    private final IntakeFeeder intake_ = IntakeFeeder.getInstance();

    public LimelightShoot() {
        
        addCommands(
            new RunCommand(vision_::setLimelightOffset).withTimeout(2)
            .andThen(() -> vision_.setLimelightOffset(0)),
    
            new InstantCommand(
                () -> shooter_.setVelocityGoal(Constants.kLongShotVelocity, false))
                .perpetually()
                .withInterrupt(() -> shooter_.spunUp()
            ),
            
            new RunCommand(intake_::shoot).withTimeout(3)
            .andThen(new InstantCommand(intake_::stopIntake))
            .andThen(new InstantCommand(shooter_::stop))
        );
    }
}
