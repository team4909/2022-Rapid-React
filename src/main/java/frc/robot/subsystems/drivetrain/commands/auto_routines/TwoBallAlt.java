package frc.robot.subsystems.drivetrain.commands.auto_routines;

import java.util.List;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.lib.bioniclib.AutoRoutineBase;
import frc.robot.subsystems.drivetrain.commands.TrajectoryFollow;
import frc.robot.subsystems.shooter.commands.AutoShot;

public class TwoBallAlt extends AutoRoutineBase {

    public TwoBallAlt() {
        super();
        addCommands(
            new RunCommand(m_intake::intake, m_intake).withTimeout(0.3),
            new TrajectoryFollow(getNextTrajectory())
            .raceWith(new RunCommand(m_intake::intake, m_intake))
            .andThen(new AutoShot(m_vision, m_shooter, m_hood).withTimeout(0.5)),

            new RunCommand(m_intake::shoot).withTimeout(1.5)
            .andThen(new InstantCommand(m_intake::stopIntake)));
    }

    protected List<Pair<String, Double>> addTrajectories() {
        return List.of(
            new Pair<String, Double>("Tarmac-A", 2.0)
        );
    }

    
}