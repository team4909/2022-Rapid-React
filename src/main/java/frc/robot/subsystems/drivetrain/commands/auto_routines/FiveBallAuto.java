package frc.robot.subsystems.drivetrain.commands.auto_routines;

import java.util.List;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.lib.bioniclib.AutoRoutineBase;
import frc.robot.subsystems.drivetrain.commands.TrajectoryFollow;
import frc.robot.subsystems.shooter.commands.AutoShot;

public class FiveBallAuto extends AutoRoutineBase {

    public FiveBallAuto() {
        addCommands(
            new PrintCommand("WHY ARENT U DOING ANYHTING"),
            new RunCommand(m_intake::intake, m_intake).withTimeout(0.3),
            new TrajectoryFollow(getTrajectory(0)).withTimeout(1)
                .raceWith(new RunCommand(m_intake::intake, m_intake))
                .andThen(new AutoShot(m_vision, m_shooter, m_hood).withTimeout(0.5)),
            new RunCommand(m_intake::shoot).withTimeout(1.5)
                .andThen(new InstantCommand(m_intake::stopIntake)),

            new TrajectoryFollow(getTrajectory(1))
                .raceWith(new RunCommand(m_intake::intake, m_intake))
                .andThen(new AutoShot(m_vision, m_shooter, m_hood).withTimeout(0.5)),
            new RunCommand(m_intake::shoot).withTimeout(1.5)
                .andThen(new InstantCommand(m_intake::stopIntake))
                .andThen(new InstantCommand(m_shooter::stop)),

            new TrajectoryFollow(getTrajectory(2))
                .raceWith(new RunCommand(m_intake::intake, m_intake)),

            new TrajectoryFollow(getTrajectory(3))
                .andThen(new AutoShot(m_vision, m_shooter, m_hood).withTimeout(0.5)),
            new RunCommand(m_intake::shoot).withTimeout(3)
                .andThen(new InstantCommand(m_intake::stopIntake))
                .andThen(new InstantCommand(m_shooter::stop))
        );
    }

    protected List<Pair<String, Double>> addTrajectories() {
        return List.of(
            new Pair<String, Double>("Tarmac-Almost-A", 2.0),
            new Pair<String, Double>("A-B", 2.5),
            new Pair<String, Double>("B-CD", 3.0),
            new Pair<String, Double>("B-CD-Reverse", 1.5)
        );
    }
    
}