package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class SnapToAngle extends CommandBase {

	private final double kP = 0.1;
	private final double kI = 0.0;
	private final double kD = 0.0012;
	private final DrivetrainSubsystem dt_;
	private final PIDController thetaController_;
	private final XboxController currentInput_;
	private double angle_ = 0d;
	private boolean fenderAngle_ = false;

	public SnapToAngle(XboxController controller, DrivetrainSubsystem requirements) {
		currentInput_ = controller;
		thetaController_ = new PIDController(kP, kI, kD);
		addRequirements(requirements);
		dt_ = requirements;
		fenderAngle_ = true;
	}

	public SnapToAngle(XboxController controller, double angle, DrivetrainSubsystem requirements) {
		currentInput_ = controller;
		thetaController_ = new PIDController(kP, kI, kD);
		angle_ = angle;
		addRequirements(requirements);
		dt_ = requirements;
	}

	@Override
	public void initialize() {

		thetaController_.setTolerance(4); //if 2.5 degrees off thats ok
		if (fenderAngle_) {
			if (currentInput_.getPOV() != -1) {
				if (angle_ > 180) angle_ -= 360;
			}
		} else {
			if (angle_ > 180) angle_ -= 360;
		}

		double gyroAngle = dt_.getGyroscopeRotation().getDegrees() % 360;
		gyroAngle += (gyroAngle > 180) ? -360 : 360;

		// Math to find shortest path
		double angleDiff = angle_ - gyroAngle;
		if (Math.abs(angleDiff) > 180) {
			if (angle_ > 0 && gyroAngle < 0) {
				angle_ -= 360;
			}
			angle_ += 360;
		}
		SmartDashboard.putNumber("Snap Goal", angle_);
		SmartDashboard.putNumber("Snap Starting", gyroAngle);


	}

	@Override
	public void execute() {
		SmartDashboard.putNumber("turnError", thetaController_.getPositionError());
		double gyroAngle = dt_.getGyroscopeRotation().getDegrees() % 360;
		if (gyroAngle > 180) gyroAngle = 360 - gyroAngle;

		//While snapping to angle, you can drive in its simplest state but without any  of the
		//extra functionality that the arguements of DefaultDriveCommand provide.
		dt_.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
				-currentInput_.getLeftY() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
				-currentInput_.getLeftX() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
				-currentInput_.getRightX() * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND + thetaController_.calculate(gyroAngle, -angle_),
				dt_.getGyroscopeRotation()
		));
	}

	@Override
	public boolean isFinished() {
		return thetaController_.atSetpoint();
	}
}