package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class leds extends SubsystemBase {
    Spark spark = new Spark(0);
    double SparkValue = -0.21;
    public static leds instance_ = null;
    public void MORECOLORS() {
        SparkValue += 0.02;
    }
    private leds() {
        spark.set(SparkValue);
   }
   public void LedIntakeStop() {
       spark.set(0.65);
   }
   public void LedIntakeGo() {
       spark.set(0.75);
   }
   public static leds getInstance() {
    if (instance_ == null) {
        instance_ = new leds();
    }
    return instance_;
}

}