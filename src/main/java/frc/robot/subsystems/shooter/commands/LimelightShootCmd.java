package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.IntakeFeeder;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.LimeLight;
import frc.robot.subsystems.vision.VisionSubsystem;

public class LimelightShootCmd extends CommandBase{

    private final IntakeFeeder intakeFeeder_;
    private final Shooter shooter_;
    private final VisionSubsystem vision_;
    private static double goal_;

    public LimelightShootCmd() {
        intakeFeeder_ = IntakeFeeder.getInstance();
        shooter_ = Shooter.getInstance();
        vision_ = VisionSubsystem.getInstance();
    }


    @Override
    public void initialize() {
        // Don't know if there's anything needed here yet 
    }

    @Override
    public void execute() {
        // Get goal from limelight, and send to the shooter as the velocity goal
        // Might want to set a default if the limelight can't calculate a goal due to a blockage or something
        goal_ = vision_.getVelocityGoal();
        shooter_.setVelocityGoal(goal_, false); 

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
