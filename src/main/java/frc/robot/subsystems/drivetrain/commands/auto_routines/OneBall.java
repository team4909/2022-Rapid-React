package frc.robot.subsystems.drivetrain.commands.auto_routines;

import java.util.List;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.lib.bioniclib.AutoRoutineBase;
import frc.robot.subsystems.drivetrain.commands.TrajectoryFollow;
import frc.robot.subsystems.shooter.commands.AutoShot;

public class OneBall extends AutoRoutineBase {

    public OneBall() {
        addCommands(
            new TrajectoryFollow(getNextTrajectory()),
            new AutoShot(m_vision, m_shooter, m_hood).withTimeout(2),
            new RunCommand(m_intake::shoot).withTimeout(2)
            .andThen(new InstantCommand(m_intake::stopIntake))
            .andThen(new InstantCommand(m_shooter::stop))
        );
    }

    protected List<Pair<String, Double>> addTrajectories() {
        return List.of(
            new Pair<String, Double>("FenderTaxi", 2.0)
        );
    }
}
