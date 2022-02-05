package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem extends SubsystemBase{
    
    public boolean isAligned;

private VisionSubsystem() {

}

public static VisionSubsystem instance = null;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

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
        double distance;
        distance = (Constants.tapeHeight-Constants.limelightHeight) / Math.tan(Math.toRadians(Constants.limelightAngle+ty.getDouble(0.0)));;
        SmartDashboard.putNumber("Distance", distance);
        return distance;
    }

    public double getGoal(){
        double goal;
        double m = 18;
        double b = 2150;
        double y = ty.getDouble(0.0);

        if(y == 0.0){
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

    public static VisionSubsystem getInstance() {
        if (instance == null) {
            instance = new VisionSubsystem();
        }

        return instance;
    }

    public void periodic() {
        if (Math.abs(getXDegrees()) <=2) {
            isAligned = true;
        } else {
            isAligned = false;
        }
        SmartDashboard.putBoolean("isAligned", isAligned);
    }

}
