package frc.robot.subsystems.drivetrain.commands.auto_routines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.commands.PathResetOdometry;
import frc.robot.subsystems.drivetrain.commands.TrajectoryFollow;
import frc.robot.subsystems.intake.IntakeFeeder;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.VisionSubsystem;

public class FiveBallAuto extends SequentialCommandGroup {
    
    IntakeFeeder intake_ = IntakeFeeder.getInstance();
    Shooter shooter_ = Shooter.getInstance();
    VisionSubsystem vision_ = VisionSubsystem.getInstance();

    public FiveBallAuto() {
        addCommands(
            new PathResetOdometry("Tarmac-A"),
            new TrajectoryFollow("Tarmac-A").withTimeout(1)
            
            
        );
    }

}
