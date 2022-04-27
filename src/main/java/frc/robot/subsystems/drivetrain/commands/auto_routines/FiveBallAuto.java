package frc.robot.subsystems.drivetrain.commands.auto_routines;

import java.util.List;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.lib.bioniclib.AutoRoutineBase;
import frc.robot.subsystems.drivetrain.commands.TrajectoryFollow;
import frc.robot.subsystems.shooter.commands.AutoShot;

public class FiveBallAuto extends AutoRoutineBase {

    public FiveBallAuto() {
        super();
        addCommands(  
            new RunCommand(m_intake::intake, m_intake).withTimeout(0.3),
            new TrajectoryFollow(getTrajectory(0)).withTimeout(2)
                .raceWith(new RunCommand(m_intake::intake, m_intake))
                .andThen(new AutoShot(m_vision, m_shooter, m_hood).withTimeout(0.5)),
            new RunCommand(m_intake::shoot).withTimeout(1.5)
                .andThen(new InstantCommand(m_intake::stopIntake)),

            new TrajectoryFollow(getTrajectory(1)).withTimeout(2.5)
                .raceWith(new RunCommand(m_intake::intake, m_intake))
                .andThen(new AutoShot(m_vision, m_shooter, m_hood).withTimeout(0.5)),
            new RunCommand(m_intake::shoot).withTimeout(1.5)
                .andThen(new InstantCommand(m_intake::stopIntake))
                .andThen(new InstantCommand(m_shooter::stop)),

            new TrajectoryFollow(getTrajectory(2)).withTimeout(3)
                .raceWith(new RunCommand(m_intake::intake, m_intake)),

            new TrajectoryFollow(getTrajectory(3)).withTimeout(1.5)
            .andThen(new AutoShot(m_vision, m_shooter, m_hood).withTimeout(0.5)),
            new RunCommand(m_intake::shoot).withTimeout(3)
                .andThen(new InstantCommand(m_intake::stopIntake))
                .andThen(new InstantCommand(m_shooter::stop))
        
        );
    }

    @Override
    protected List<String> addTrajectories() {
        return List.of("Tarmac-Almost-A", "A-B", "B-CD", "B-CD-Reverse");
    }
    
}