package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeFeeder extends SubsystemBase {

    private enum IntakeState {
        kIdle("Idle"),
        kIdleFeeder("Idle Feeder"),
        kIntakeFirst("Intake First"),
        kIntakeSecond("Intake Second"),
        kShootBalls("Shooting Both Balls"),
        kReverseWrongBall("Reverse Wrong Ball");

        public String name;

        private IntakeState(String n) {
            this.name = n;
        }

        public String toString() {
            return this.name;
        }
    };

    private enum BallCount{
        kZero, kOne, kTwo
    }

    // Object instance
    private static IntakeFeeder instance_ = null;

    private CANSparkMax intakeWheels_;
    private CANSparkMax centeringWheel_;
    private CANSparkMax feederWheel_;

    private final DoubleSolenoid intakeExtension_;

    private final DigitalInput lowSensor_; // One closest to intake
    private final DigitalInput highSensor_; // One closes to shooter
    private final DigitalInput incomingSensor_; // To know when to run feader

    private static boolean intakeSolenoidState_ = false; // Assuming false is the default state (intact retracted)

    private static IntakeState currentState_;
    private static IntakeState lastState_;

    private static BallCount ballsHeld_;

    private static int fallingEdges = 0;
    private static boolean lastEdgeHigh = false;
    private static boolean rumble_ = false;



    private IntakeFeeder() {
        intakeWheels_ = new CANSparkMax(18, MotorType.kBrushless);
        intakeWheels_.restoreFactoryDefaults();
        centeringWheel_ = new CANSparkMax(16, MotorType.kBrushless);
        feederWheel_ = new CANSparkMax(17, MotorType.kBrushless);

        intakeExtension_ = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
    
        lowSensor_ = new DigitalInput(0);
        highSensor_ = new DigitalInput(1);
        incomingSensor_ = new DigitalInput(2);

        currentState_ = IntakeState.kIdle;
        lastState_ = IntakeState.kIdle;

        ballsHeld_ = BallCount.kZero;
    }

    public static IntakeFeeder getInstance() {
        if (instance_ == null) {
            instance_ = new IntakeFeeder();
        } 
        return instance_;
    }

    public void intake() {
        intakeSolenoidState_ = true;
        switch (ballsHeld_) {
            case kZero:
                currentState_ = IntakeState.kIntakeFirst;
                break;
            case kOne:
                currentState_ = IntakeState.kIdleFeeder;
                break;
            case kTwo:
            default:
                currentState_ = IntakeState.kIdle;
                break;
        }
    }

    // Stop all rollers in the intake
    public void stopIntake() {
        currentState_ = IntakeState.kIdle;
        intakeSolenoidState_ = false; // intake in

    }

    // Tell the feeder system to run all the motors to shoot
    public void shoot() {
        currentState_ = IntakeState.kShootBalls;
    }

    // Manually toggle the intake moving in and out (might not be needed, but nice to have)
    public void toggleIntakeExtension() {
        intakeSolenoidState_ = !intakeSolenoidState_;
    }

    // Right now map this to a command/button. can make automatic with the color sensor later
    // Just want to test that reversing works
    public void reverseIntake() {
        currentState_ = IntakeState.kReverseWrongBall;
        intakeSolenoidState_ = true; // intake in

    }

    public boolean getRumble() {
        return rumble_;
    }

    @Override
    public void periodic() {

        // Get all the readings from the sensors each tick so we know what's changed
        boolean firstBallSeen = lowSensor_.get();
        boolean feederBallSeen = highSensor_.get();
        boolean incomingSeen = incomingSensor_.get();
        // Set the solenoid to be in it's state
        // Because it's a single acting solenoid it needs to be continously set
        // otherwise it reverts to the default state
        intakeExtension_.set(intakeSolenoidState_ ? Value.kReverse : Value.kForward);
        
        switch (currentState_) {
            case kIdle:
                // Everything is idle, nothing runs
                intakeWheels_.set(0.0);
                centeringWheel_.set(0.0);
                feederWheel_.set(0.0);
                break;
            case kIdleFeeder:
                // Only go to this state between intaking balls one and two. Could be better named
                // to "IntermediateIntake" or something if that's better
                feederWheel_.set(0.0);
                // Continue running the intaking and centering wheels until there's another ball seen
                intakeWheels_.setVoltage(Constants.kIntakeForwardVoltage);
                centeringWheel_.setVoltage(Constants.kCenteringWheelForwardVoltage);
                if (incomingSeen) {
                    // Set state to get the next ball
                    currentState_ = IntakeState.kIntakeSecond;
                }
                break;
            case kIntakeFirst:
                // Run everything to get first ball into position
                intakeWheels_.setVoltage(Constants.kIntakeForwardVoltage);
                centeringWheel_.setVoltage(Constants.kCenteringWheelForwardVoltage);
                feederWheel_.setVoltage(Constants.kFeederFeedingVoltage);
                // if (firstBallSeen) {
                //     // Want to keep running the intake, but not the feeder
                //     currentState_ = IntakeState.kIdleFeeder;
                //     ballsHeld_ = BallCount.kOne;
                //     // Not doing this yet, but for the future driver feedback
                //     rumble_ = true;
                // }
                break;
            case kIntakeSecond:
                // Keep running intake, but now know the second ball is at least in the robot
                intakeWheels_.setVoltage(Constants.kIntakeForwardVoltage);
                centeringWheel_.setVoltage(Constants.kCenteringWheelForwardVoltage);
                // Have already checked to make sure that the fender wheel should be running                
                feederWheel_.setVoltage(Constants.kFeederFeedingVoltage);
                // Stop feeder if either two balls reach the position
                // Might need more complex logic, depends on the geometry but should be fine for testing
                if (firstBallSeen || feederBallSeen ) {
                    currentState_ = IntakeState.kIdleFeeder;
                    ballsHeld_ = BallCount.kTwo;
                    // Not doing this yet, but for the future driver feedback
                    rumble_ = true;
                }
                break;
            case kShootBalls:
                intakeWheels_.set(0.0);
                centeringWheel_.setVoltage(Constants.kCenteringWheelForwardVoltage);
                feederWheel_.setVoltage(Constants.kFeederShootingVoltage);
                // TODO due to change in sensors. Ignore for now in testing
                // Don't need to count them right now
                // if(countFalling(firstBallSeen) >= 2) {
                //     fallingEdges = 0;
                //     ballsHeld_ = BallCount.kZero;
                //     currentState_ = IntakeState.kIdle;
                // }
                
                break;
            case kReverseWrongBall:
                // Reverse the motors
                intakeWheels_.setVoltage(Constants.kIntakeReverseVoltage);
                centeringWheel_.setVoltage(Constants.kCenteringWheelReverseVoltage);
                feederWheel_.setVoltage(Constants.kFeederReverseVoltage);
                break;
            default:
                break;
        }
        SmartDashboard.putString("Intake State", currentState_.name);

        // not using this now but probably will want to down the line
        // for knowing what state the robot previously was in
        lastState_ = currentState_;
    }

    // TODO change because of the new sensors
    private int countFalling(boolean seen) {
        if (lastEdgeHigh && lastEdgeHigh != seen) {
            fallingEdges++;
            lastEdgeHigh = false;
        }

        return fallingEdges;
    }

}