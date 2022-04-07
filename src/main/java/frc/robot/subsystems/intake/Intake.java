package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    /**
     * States enum for all the different states the intake could be in
     * States: In, Out, and Calibrate (Zeroing)
     */
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

    /**
     * Intake Constructor
     */
    private Intake(){
        // Neo 550
        intakeMotor_ = new CANSparkMax(Constants.Intake.INTAKE_MOTOR, MotorType.kBrushless);
        // Sets current limit from constants
        intakeMotor_.setSmartCurrentLimit(Constants.Intake.CURRENT_LIMIT);

        // Position PID Controller
        positionController_ = intakeMotor_.getPIDController();

        positionController_.setP(Constants.Intake.POSITION_KP);
        positionController_.setI(Constants.Intake.POSITION_KI);
        positionController_.setD(Constants.Intake.POSITION_KD);
        positionController_.setFF(Constants.Intake.POSITION_KF);

        // Max + Min voltage, intake can't break the robot
        positionController_.setOutputRange(-Constants.Intake.MAX_VOLTAGE, Constants.Intake.MAX_VOLTAGE);

        // Starting state is IN
        currentState_ = lastState_ = IntakeStates.IN;

    }

    @Override
    public void periodic() {
        // Checks to make sure the state has changed
        if(currentState_ != lastState_){
            switch(currentState_){
                case OUT:
                    // PID Refrence set to out setpoint, ~30
                    positionController_.setReference(Constants.Intake.OUT_SETPOINT, ControlType.kPosition);
                    break;
                case IN:
                    // PID Refrence set to zero
                    positionController_.setReference(0, ControlType.kPosition);
                    break;
                case CALIBRATE:
                    // Calls the calibrate method
                    calibrateIntake();
                    break;
            }
        }

        SmartDashboard.putString("Intake State", currentState_.stateName);

        lastState_ = currentState_;

    }

    private void calibrateIntake(){
        // Runs the intake backwards for .75 seconds, and then sets the encoder position to 0
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
        // if(instance_ == null) instance_ = new Intake();
        // We should use this cause it's cooler
        return (instance_ == null) ? instance_ = new Intake() : instance_;
    }

    
}
