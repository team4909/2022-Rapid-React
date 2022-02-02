package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLight {
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

public double getDistance(){
    double distance;
    double angle;

    // double distance = (targetHeight - cameraHeight) / Math.tan(Math.toRadians(CameraAngle + YOffset))
    angle =  Math.tan(Math.toRadians(18.75+ty.getDouble(0.0)));
    distance = (103-27) / angle;
    SmartDashboard.putNumber("Distance", distance);
    SmartDashboard.putNumber("angle", angle);
    return distance;

}

public double getGoal(){
    double goal;
    double y = ty.getDouble(0.0);

    if(y == 0.0){
        return 0;
    }

    goal = 18 * getDistance() + 2150;
    SmartDashboard.putNumber("Goal", goal);
    return goal;
}

public double getXDegrees(){
    double x = tx.getDouble(0.0);
    return x;
}

}
