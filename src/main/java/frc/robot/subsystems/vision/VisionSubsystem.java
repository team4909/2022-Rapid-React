package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.BionicController;

import java.util.Map;

import org.opencv.video.Video;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem extends SubsystemBase{
    // Made into privates
    private boolean isAligned_;
    private double lastDistance_;

    private double limelightOffset;
    private double firstError;

    public boolean isAligned;
    boolean toggle = true;

    private UsbCamera frontCamera_;
    private VideoSink server_;
    private Thread cameraThread;

private VisionSubsystem() {


    UsbCamera camera = CameraServer.startAutomaticCapture();
    camera.setResolution(240, 128);
    camera.setFPS(30);
    Shuffleboard.getTab("Driver").add(camera)
        .withPosition(8, 0)
        .withSize(5, 4)
        .withWidget("Camera Stream")
        .withProperties(Map.of("Rotation", "HALF", "Show controls", true));
    isAligned_ = false;
    lastDistance_ = 0.0;
    firstError = this.getXDegrees();
}

public static VisionSubsystem instance_ = null;


    // Network table values
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");


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

    public void setLimelightOffset() {

        double offset = -this.getXDegrees();
        double angularSpeed = (offset * Constants.GOAL_ALIGN_KP + Math.abs(offset - firstError) * Constants.GOAL_ALIGN_KD) * 2; //TODO make this a constant pls

        if (this.isAligned() == true) {
            limelightOffset = 0;
        } else {
            if (this.getXDegrees() <= 0)
                limelightOffset = angularSpeed;

            if (this.getXDegrees() >= 0) 
                limelightOffset = angularSpeed;
        }
    }

    public void setLimelightOffset(double value) {
        limelightOffset = value;
    }

    /**
     * @return chasis speed, not angle in measure
     */
    public double getLimelightOffset() {
        return this.limelightOffset;
    }

    public boolean isAligned() {
        return isAligned_;
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
        SmartDashboard.putNumber("ofset speed", limelightOffset);

    }

    

}
