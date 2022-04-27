package frc.robot.subsystems.drivetrain.commands.auto_routines;

import java.util.List;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.lib.bioniclib.AutoRoutineBase;
import frc.robot.subsystems.drivetrain.commands.TrajectoryFollow;
import frc.robot.subsystems.shooter.commands.AutoShot;

public class TwoBall extends AutoRoutineBase {

    public TwoBall() {
        addCommands(
            new RunCommand(m_intake::intake, m_intake).withTimeout(1.5),
            new TrajectoryFollow(getTrajectory(0)).withTimeout(1.7),

            new InstantCommand(m_intake::stopIntake)
                .andThen(new AutoShot(m_vision, m_shooter, m_hood).withTimeout(2)),

            new RunCommand(m_intake::shoot).withTimeout(3)
                .andThen(new InstantCommand(m_intake::stopIntake))
                .andThen(new InstantCommand(m_shooter::stop))
                
        );
    }

    @Override
    protected List<String> addTrajectories() {
        return List.of("TarmacN-E");
    }
    
}
