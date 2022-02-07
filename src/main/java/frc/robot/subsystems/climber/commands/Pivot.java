package frc.robot.subsystems.climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.Climber;

public class Pivot extends CommandBase {

    private double angle_;
    private double goal_;
    private final Climber climber_ = Climber.getInstance();

    public Pivot(double angle) {
        angle_ = angle;
    }

    @Override
    public void initialize() {
        goal_ = angle_; //TODO Convert degrees to ticks
    }

    public void execute() {
        climber_.setPivotGoal(goal_);
    }

    @Override
    public boolean isFinished() {
        return climber_.m_isAtPivotGoal;
    }
    
}