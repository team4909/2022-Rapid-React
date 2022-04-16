package frc.robot.utils;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDS extends SubsystemBase{
    
    public Spark LEDController;
    private static LEDS m_instance = null;

    private LEDS() {
        LEDController = new Spark(9);
    }    

    public void setLEDsRed(){
        LEDController.set(0.61);
    }

    public void setLEDsGreen() {
        LEDController.set(0.73);
    }


    public static LEDS getInstance() {
        if (m_instance == null) {
            m_instance = new LEDS();
        }

        return m_instance;
    }


}
