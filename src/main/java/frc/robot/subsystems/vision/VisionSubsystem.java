package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.BionicController;

import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import com.fasterxml.jackson.annotation.JacksonInject.Value;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem extends SubsystemBase{
    
    public boolean isAligned;
    boolean toggle = true;

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

    public void setPipeline() {
        if(toggle == true){
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
            toggle = false;
        }
        else{
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
            toggle = true;
        }
    }

    public void checkRumble(BionicController controller){
        NetworkTableEntry tv = table.getEntry("tv");
        double check = tv.getDouble(0.0);
        if(check == 0.0){
            controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
            controller.setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
            SmartDashboard.putString("rumble", "none");
        }
        else 
        {
            if(Math.abs(getXDegrees()) < 2){            
                controller.setRumble(GenericHID.RumbleType.kLeftRumble, 1.0);
                controller.setRumble(GenericHID.RumbleType.kRightRumble, 1.0);
                SmartDashboard.putString("rumble", "strong");
            }
            else if(Math.abs(getXDegrees()) > 2){
                controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
                controller.setRumble(GenericHID.RumbleType.kRightRumble, 0.5);
                SmartDashboard.putString("rumble", "light");
            }
        }
    }

    public void endRumble(BionicController controller){
        controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
        controller.setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
        SmartDashboard.putString("rumble", "ended");
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