package frc.robot.subsystems.drivetrain.commands.auto_routines;

import java.util.List;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.lib.bioniclib.AutoRoutineBase;
import frc.robot.subsystems.drivetrain.commands.TrajectoryFollow;
import frc.robot.subsystems.shooter.commands.AutoShot;

public class TwoBallDisrupt extends AutoRoutineBase {

    public TwoBallDisrupt() {
        super();
        addCommands(
            new RunCommand(m_intake::intake, m_intake)
            .alongWith(new TrajectoryFollow(getTrajectory(0))).withTimeout(1.7),

            new InstantCommand(m_intake::stopIntake)
            .andThen(new AutoShot(m_vision, m_shooter, m_hood).withTimeout(2)),

            new RunCommand(m_intake::shoot).withTimeout(2)
            .andThen(new InstantCommand(m_intake::stopIntake))
            .andThen(new InstantCommand(m_shooter::stop)),

            new RunCommand(m_intake::intake, m_intake)
            .alongWith(new TrajectoryFollow(getTrajectory(1))).withTimeout(5.2),

            new InstantCommand(m_intake::stopIntake),

            new RunCommand(m_intake::reverseIntake).withTimeout(1.5)
            .andThen(new InstantCommand(m_intake::stopIntake))
            
        );
    }

    @Override
    protected List<String> addTrajectories() {
        List<String> t = List.of("TarmacN-E", "E-FGSpit");
        return t;
    }
}