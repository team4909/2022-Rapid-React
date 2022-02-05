package frc.robot.subsystems.climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.Climber;

public class Pivot extends CommandBase {

    private double m_angle;
    private double m_goal;
    private Climber m_climber = Climber.getInstance();

    public Pivot(double angle) {
        m_angle = angle;
    }

    @Override
    public void initialize() {
        m_goal = m_angle; //TODO Convert degrees to ticks
    }

    public void execute() {
        m_climber.setPivotGoal(m_goal);
    }

    @Override
    public boolean isFinished() {
        return m_climber.isAtPivotGoal;
    }
    
}