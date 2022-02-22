package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class SnapToAngle extends CommandBase {

    private double angle_;
    private DrivetrainSubsystem dt_;
    private PIDController thetaController_;
    
    private final double kP = 0.05;
    private final double kI = 0d;
    private final double kD = 0.0005;

    public SnapToAngle(double angle, DrivetrainSubsystem requirements) {
        angle_ = angle;

        thetaController_ = new PIDController(kP, kI, kD);
        addRequirements(requirements);
        dt_ = requirements;
    }

    public void initialize() {

        if (angle_ > 180) angle_ -= 360;
        thetaController_.setSetpoint(angle_);
        thetaController_.setTolerance(5);
    }

    public void execute() {
        SmartDashboard.putNumber("turnError", thetaController_.getPositionError());
        SmartDashboard.putNumber("goalAng", angle_);
        double gyroAngle = dt_.getGyroscopeRotation().getDegrees() % 360;
        
        if (gyroAngle > 180) gyroAngle -= 360;
        
        dt_.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0d, 0d, thetaController_.calculate(-gyroAngle), dt_.getGyroscopeRotation()));

    }
    @Override
    public boolean isFinished() {
        return thetaController_.atSetpoint();
    }
}