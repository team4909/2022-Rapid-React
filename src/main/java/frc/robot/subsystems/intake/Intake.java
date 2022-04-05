package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    public enum IntakeStates{

        IN("IN"),
        OUT("OUT"),
        HOLD("HOLD");

        String stateName;

        private IntakeStates(String name){
            this.stateName = name;
        }

        public String toString(){
            return stateName;
        }
    }

    private static Intake instance_;

    private CANSparkMax intakeMotor_;
    private SparkMaxPIDController positionController_;

    private IntakeStates currentState_;
    private IntakeStates lastState_;
    
    private Intake(){
        intakeMotor_ = new CANSparkMax(Constants.Intake.INTAKE_MOTOR, MotorType.kBrushless);
        positionController_ = intakeMotor_.getPIDController();

        positionController_.setP(Constants.Intake.POSITION_KP);
        positionController_.setI(Constants.Intake.POSITION_KI);
        positionController_.setD(Constants.Intake.POSITION_KD);

        positionController_.setOutputRange(0, 8);

        currentState_ = lastState_ = IntakeStates.IN;

    }

    @Override
    public void periodic() {
        if(currentState_ != lastState_){
            switch(currentState_){
                case OUT:
                    positionController_.setReference(Constants.Intake.OUT_SETPOINT, ControlType.kPosition);
                    break;
                case IN:
                    positionController_.setReference(0, ControlType.kPosition);
                    break;
                case HOLD:
                    positionController_.setReference(Constants.Intake.HOLD_VOLTAGE, ControlType.kVoltage);
                    break;
                default:
                    currentState_ = IntakeStates.IN;
                    break;
            }
        }

        lastState_ = currentState_;

    }

    public static Intake getInstance(){
        if(instance_ == null) return new Intake();
        return instance_;
    }

    
}
