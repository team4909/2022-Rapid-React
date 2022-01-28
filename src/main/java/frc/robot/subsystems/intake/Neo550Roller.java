package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class Neo550Roller implements Sendable {

    private final CANSparkMax roller_;

    public Neo550Roller(int canID, boolean inverted) {
        roller_ = new CANSparkMax(canID, MotorType.kBrushless);
        roller_.setInverted(inverted);
    }

    public void stop() {
        roller_.set(0.0);
    }

    public void run(double voltage) {
        roller_.setVoltage(voltage);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // TODO Auto-generated method stub
        
    }
    
}
