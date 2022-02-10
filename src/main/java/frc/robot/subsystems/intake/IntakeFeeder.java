package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
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
    private static IntakeFeeder instance_;

    private final Neo550Roller intakeWheels_;
    private final Neo550Roller centeringWheel_;
    private final Neo550Roller feederWheel_;

    private final Solenoid intakeExtension_;

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
        intakeWheels_ = new Neo550Roller(20, false);
        centeringWheel_ = new Neo550Roller(21, false);
        feederWheel_ = new Neo550Roller(22, false);

        intakeExtension_ = new Solenoid(PneumaticsModuleType.REVPH, 1);
    
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
        intakeSolenoidState_ = true;

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
    }

    public boolean getRumble() {
        return rumble_;
    }

    @Override
    public void periodic() {

        boolean firstBallSeen = lowSensor_.get();
        boolean feederBallSeen = highSensor_.get();
        boolean incomingSeen = incomingSensor_.get();
        intakeExtension_.set(intakeSolenoidState_);
        
        switch (currentState_) {
            case kIdle:
                intakeWheels_.stop();
                centeringWheel_.stop();
                feederWheel_.stop();
                intakeSolenoidState_ = false;
                break;
            case kIdleFeeder:
                feederWheel_.stop();
                // Continue running the intaking and centering wheels until there's another ball seen
                intakeWheels_.run(Constants.kIntakeForwardVoltage);
                centeringWheel_.run(Constants.kCenteringWheelForwardVoltage);
                if (incomingSeen) {
                    // This will set the state based on the number of balls held
                    currentState_ = IntakeState.kIntakeSecond;
                }
                break;
            case kIntakeFirst:
                intakeWheels_.run(Constants.kIntakeForwardVoltage);
                centeringWheel_.run(Constants.kCenteringWheelForwardVoltage);
                feederWheel_.run(Constants.kFeederFeedingVoltage);
                if (firstBallSeen) {
                    currentState_ = IntakeState.kIdleFeeder;
                    ballsHeld_ = BallCount.kOne;
                    rumble_ = true;
                }
                break;
            case kIntakeSecond:
                intakeWheels_.run(Constants.kIntakeForwardVoltage);
                centeringWheel_.run(Constants.kCenteringWheelForwardVoltage);
                // Have already checked to make sure that the fender wheel should be running                
                feederWheel_.run(Constants.kFeederFeedingVoltage);
                // Stop feeder if 
                if (firstBallSeen || feederBallSeen ) {
                    currentState_ = IntakeState.kIdleFeeder;
                    ballsHeld_ = BallCount.kTwo;
                    rumble_ = true;
                }
                break;
            case kShootBalls:
                intakeWheels_.stop();
                centeringWheel_.run(Constants.kCenteringWheelForwardVoltage);
                feederWheel_.run(Constants.kFeederShootingVoltage);
                // TODO due to change in sensors. Ignore for now in testing
                // if(countFalling(firstBallSeen) >= 2) {
                //     fallingEdges = 0;
                //     ballsHeld_ = BallCount.kZero;
                //     currentState_ = IntakeState.kIdle;
                // }
                
                break;
            case kReverseWrongBall:
                intakeWheels_.run(Constants.kIntakeReverseVotlage);
                centeringWheel_.run(Constants.kCenteringWheelReverseVotlage);
                break;
            default:
                break;
        }

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