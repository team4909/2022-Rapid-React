package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class SnapToAngle extends CommandBase {

    private double angle_= 0;
    private DrivetrainSubsystem dt_;
    private PIDController thetaController_;
    private XboxController controller_;
    
    private final double kP = 0.12;
    private final double kI = 0.0;
    private final double kD = 0.0012;

    public SnapToAngle(XboxController controller, DrivetrainSubsystem requirements) {
        controller_ = controller;
        thetaController_ = new PIDController(kP, kI, kD);
        addRequirements(requirements);
        dt_ = requirements;
    }

    public SnapToAngle(double angle, DrivetrainSubsystem requirements) {
        thetaController_ = new PIDController(kP, kI, kD);
        angle_ = angle;
        addRequirements(requirements);
        dt_ = requirements;
    }

    @Override
    public void initialize() {

        thetaController_.setTolerance(4); //if 2.5 degrees off thats ok
        SmartDashboard.putNumber("goalAng", angle_);
    }

    @Override
    public void execute() {
        if (controller_ != null) {
            if (controller_.getPOV() != -1) {
                if (angle_ > 180) angle_ -= 360;   
            }
        } else {
            if (angle_ > 180) angle_ -= 360;
        }

        SmartDashboard.putNumber("turnError", thetaController_.getPositionError());
        double gyroAngle = dt_.getGyroscopeRotation().getDegrees() % 360;
        if (gyroAngle > 180) gyroAngle = 360 - gyroAngle;
        dt_.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0d, 0d, thetaController_.calculate(gyroAngle, -angle_), dt_.getGyroscopeRotation()));
    }
    @Override
    public boolean isFinished() {
        return thetaController_.atSetpoint();
    }
}