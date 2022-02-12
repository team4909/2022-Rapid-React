package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.IntakeFeeder;
import frc.robot.subsystems.shooter.Shooter;

public class ShootCmd extends CommandBase{

    private final IntakeFeeder intakeFeeder_;
    private final Shooter shooter_;
    private static double goal_;

    public ShootCmd(double goal) {
        intakeFeeder_ = IntakeFeeder.getInstance();
        shooter_ = Shooter.getInstance();
        goal_ = goal;
    }


    @Override
    public void initialize() {
        // Don't know if there's anything needed here yet 
        // Just reset the goal though in case
        
        
    }

    @Override
    public void execute() {
        shooter_.setVelocityGoal(goal_); 

        if (shooter_.spunUp()) {
        // Just make sure that the intake runs for the shot
            intakeFeeder_.shoot();
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        intakeFeeder_.stopIntake();
        shooter_.stop();

    }
    
}
