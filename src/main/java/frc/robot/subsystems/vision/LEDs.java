package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase{
    
    public Spark LEDController;

    public LEDs() { 
        super();
        LEDController = new Spark(0);
        LEDController.set(0.75);
        SmartDashboard.putNumber("LEDcolor", 0.00);
    }    




    ////////////////////////////////
    ///         Autons           ///
    ////////////////////////////////

    public void generalMovement(){
        // if one ball
        LEDController.set(-0.07); //Strobe Gold

        //if two ball
        LEDController.set(0.67); //Solid Gold
    }

    public void AlignAndShoot(){
        // if Not lined up with hub
        setLEDsRED();
        //break

        //if Shooter Up to Speed
        setLEDsBlueFlash();
        //break

        //if Shooter ALIGNED & UP TO SPEED
        setLEDsBlue();
        //break
    }
    

    public void setLEDsRED(){ //Not Lined Up with HUB
        LEDController.set(0.61);
    }

    public void setLEDsBlueFlash(){ // Shooter NOT up to speed
        LEDController.set(-0.09);
    }

    public void setLEDsBlue(){ //Aligned & Up to Speed
        LEDController.set(0.87);
    }

    @Override
    public void periodic() {
        double x = SmartDashboard.getNumber("LEDcolor", 0.01);
        System.out.println(x);
        LEDController.set(x);
    }

}
