package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.IntakeFeeder;
import frc.robot.subsystems.shooter.Shooter;

public class ShootCmd extends CommandBase{

    private final IntakeFeeder intakeFeeder_;
    private final Shooter shooter_;
    private static double goal_;
    private static boolean hoodUp_;
    private static Timer timer_;

    public ShootCmd(double goal, boolean hoodUp) {
        intakeFeeder_ = IntakeFeeder.getInstance();
        shooter_ = Shooter.getInstance();
        goal_ = goal;
        hoodUp_ = hoodUp;
        timer_ = new Timer();
    }


    @Override
    public void initialize() {
        // Don't know if there's anything needed here yet 
        // Just reset the goal though in case
        timer_.stop();
        timer_.reset();
        
    }

    @Override
    public void execute() {
        shooter_.setVelocityGoal(goal_, hoodUp_); 

        // if (shooter_.spunUp() && timer_.get() == 0) {
            // timer_.start();
        // Just make sure that the intake runs for the shot
        // }    
        // if (timer_.get() > 1.5) {
            if (shooter_.spunUp())
                intakeFeeder_.shoot();
        // }

        SmartDashboard.putNumber("Timer time", timer_.get());
        
    }

    @Override
    public void end(boolean interrupted) {
        intakeFeeder_.stopIntake();
        shooter_.stop();
        timer_.stop();
        timer_.reset();
    }
    
}
