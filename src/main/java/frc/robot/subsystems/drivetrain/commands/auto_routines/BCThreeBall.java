/**
 * This auto will collect and shoot three cargo
 * only in BattleCry eliminations where there is
 * an extra cargo.
 */

 package frc.robot.subsystems.drivetrain.commands.auto_routines;

import java.util.List;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.lib.bioniclib.AutoRoutineBase;
import frc.robot.subsystems.drivetrain.commands.TrajectoryFollow;
import frc.robot.subsystems.shooter.commands.AutoShot;

 public class BCThreeBall extends AutoRoutineBase {

    public BCThreeBall() {
        addCommands(
            new InstantCommand(m_intake::intake, m_intake),
            new TrajectoryFollow(getNextTrajectory())
                .andThen(new AutoShot(m_vision, m_shooter, m_hood).withTimeout(2)),
            new RunCommand(m_intake::shoot).withTimeout(2)
                .andThen(m_intake::stopIntake, m_intake),

            new InstantCommand(m_intake::intake, m_intake),
            new TrajectoryFollow(getNextTrajectory())
                .andThen(new AutoShot(m_vision, m_shooter, m_hood).withTimeout(2)),
            new RunCommand(m_intake::shoot).withTimeout(2)
                .andThen(m_intake::stopIntake, m_intake)
                .andThen(m_shooter::stop)
        );
    }

    protected List<Pair<String, Double>> addTrajectories() {
        return List.of(
            new Pair<String, Double>("TarmacN-E", 1.7),
            new Pair<String, Double>("E-MysteryBall", 2.1)
            
        );
    }
 
     
 }