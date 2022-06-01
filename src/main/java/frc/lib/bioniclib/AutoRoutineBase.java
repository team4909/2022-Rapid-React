package frc.lib.bioniclib;

import java.util.List;

import com.fasterxml.jackson.databind.ser.impl.IndexedStringListSerializer;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.commands.PathResetOdometry;
import frc.robot.subsystems.intake.IntakeFeeder;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;

public abstract class AutoRoutineBase extends SequentialCommandGroup {

    private List<Pair<String, Double>> trajectories; //<Name, Timeout>
    private int m_selectedTrajectory;

    protected IntakeFeeder m_intake;
    protected Vision m_vision;
    protected Hood m_hood;
    protected Shooter m_shooter;
    private Pair<String, Double> trajectory;
    
    protected AutoRoutineBase() {
        m_intake = IntakeFeeder.getInstance();
        m_vision = Vision.getInstance();
        m_hood = Hood.getInstance();
        m_shooter = Shooter.getInstance();
        // addRequirements(m_intake, m_vision, m_hood, m_shooter);

        m_selectedTrajectory = 0;
        trajectories = addTrajectories();
        addCommands(
            // trajectories != null ? new PathResetOdometry(trajectories.get(0).getFirst()) : null,
            // new InstantCommand(m_intake::resetBallCount, m_intake)
        );

    }

    protected abstract List<Pair<String, Double>> addTrajectories();

    //@TODO Try and write a unit test for this
    protected Pair<String, Double> getNextTrajectory() {
        System.out.println("\n\n\n\n\nAWOIEHOIAW " + m_selectedTrajectory);
        try { trajectory = getTrajectory(m_selectedTrajectory++); } 
        catch (IndexOutOfBoundsException e) { 
            trajectory = getTrajectory(trajectories.size() - 1); 
            e.printStackTrace();
        }
        SmartDashboard.putString("currTraj", trajectory.getFirst());
        return trajectory;
    }

    protected Pair<String, Double> getTrajectory(int index) {
        Pair<String, Double> t = null;
        try {
            t = trajectories.get(index);
        } catch (IndexOutOfBoundsException e) {
            e.printStackTrace();
            System.out.println(index);
        }
        return t;
    }
    
}
