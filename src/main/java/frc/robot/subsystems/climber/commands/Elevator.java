package frc.robot.subsystems.climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.Climber;

public class Elevator extends CommandBase {

    private double inches_;
    private double goal_;
    private final Climber climber_ = Climber.getInstance();

    public Elevator(double inches) {
        inches_ = inches;
    }

    @Override
    public void initialize() {
        goal_ = inches_; // TODO Convert degrees to ticks
    }

    public void execute() {
        climber_.setPivotGoal(goal_);
    }

}