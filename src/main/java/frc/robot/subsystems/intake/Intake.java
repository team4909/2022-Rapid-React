package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    public enum IntakeStates{

        IN("IN"),
        OUT("OUT"),
        CALIBRATE("CALIBRATE");

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
        intakeMotor_.setSmartCurrentLimit(Constants.Intake.CURRENT_LIMIT);

        positionController_ = intakeMotor_.getPIDController();

        positionController_.setP(Constants.Intake.POSITION_KP);
        positionController_.setI(Constants.Intake.POSITION_KI);
        positionController_.setD(Constants.Intake.POSITION_KD);
        positionController_.setFF(Constants.Intake.POSITION_KF);

        positionController_.setOutputRange(-Constants.Intake.MAX_VOLTAGE, Constants.Intake.MAX_VOLTAGE);

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
                case CALIBRATE:
                    calibrateIntake();
                    break;
            }
        }

        SmartDashboard.putString("State", currentState_.stateName);

        lastState_ = currentState_;

    }

    private void calibrateIntake(){
        new RunCommand(() -> positionController_.setReference(-0.2, ControlType.kDutyCycle), this)
        .withTimeout(0.75)
        .andThen(new InstantCommand(() -> {intakeMotor_.getEncoder().setPosition(0);})).schedule();
    }

    public void intakeOut(){
        currentState_ = IntakeStates.OUT;
    }

    public void intakeIn(){
        currentState_ = IntakeStates.IN;
    }

    public void intakeZero(){
        currentState_ = IntakeStates.CALIBRATE;
    }

    public static Intake getInstance(){
        if(instance_ == null) instance_ = new Intake();
        return instance_;
    }

    
}
