package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem extends SubsystemBase{
    // Made into privates
    private boolean isAligned_;
    private double lastDistance_;
    
    // Moved up here to be at the top
    private static VisionSubsystem instance_ = null;

    // Network table values
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    public VisionSubsystem() {
        // Default initialization
        isAligned_ = false;
        lastDistance_ = 0.0;
    }

    public static VisionSubsystem getInstance() {
        if (instance_ == null) {
            instance_ = new VisionSubsystem();
        }

        return instance_;
    }

    public void getLimeLight() {
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);

        SmartDashboard.putNumber("LimeLightX", x);
        SmartDashboard.putNumber("LimeLightY", y);
        SmartDashboard.putNumber("LimeLightArea", area);
    }

    //LED Power is 60

    public double getDistance(){
        return lastDistance_;
    }

    public double getVelocityGoal(){
        double goal;
        // Put these as constants at some point
        double m = 18;
        double b = 2150;
        double y = ty.getDouble(0.0);

        if (y == 0.0){
            return 0;
        }

        goal = m * getDistance() + b; // Linear equation, goal = mx+b
        SmartDashboard.putNumber("Goal", goal);
        return goal;
    }

    public double getXDegrees(){
        double x = tx.getDouble(0.0);
        return x;
    }

    public boolean isAligned() {
        return isAligned_;
    }

    public void periodic() {
        // Update the last values
        lastDistance_ = (Constants.tapeHeight-Constants.limelightHeight) / Math.tan(Math.toRadians(Constants.limelightAngle+ty.getDouble(0.0)));;
        getLimeLight();

        if (Math.abs(getXDegrees()) <= 2) {
            isAligned_ = true;
        } else {
            isAligned_ = false;
        }
        SmartDashboard.putBoolean("isAligned", isAligned_);
        SmartDashboard.putNumber("Distance", lastDistance_);

    }

}
