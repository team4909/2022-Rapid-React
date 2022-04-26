package frc.robot.subsystems.drivetrain.commands.auto_routines;

import java.util.List;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.lib.bioniclib.AutoRoutineBase;
import frc.robot.subsystems.drivetrain.commands.TrajectoryFollow;
import frc.robot.subsystems.intake.IntakeFeeder;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.AutoShot;

public class OneBall extends AutoRoutineBase {

    private final Shooter shooter_ = Shooter.getInstance();
    private final IntakeFeeder intake_ = IntakeFeeder.getInstance();

    public OneBall() {
        addCommands(
            new TrajectoryFollow(getTrajectory(0)).withTimeout(2),
            new AutoShot(m_vision, m_shooter, m_hood).withTimeout(2),
            new RunCommand(m_intake::shoot).withTimeout(2)
            .andThen(new InstantCommand(m_intake::stopIntake))
            .andThen(new InstantCommand(m_shooter::stop))
        );
        
    }

    @Override
    protected List<String> addTrajectories() {
        List<String> t = List.of("FenderTaxi");
        return t;
    }
}
