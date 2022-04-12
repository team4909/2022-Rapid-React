package frc.robot.subsystems.shooter;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Hood extends SubsystemBase {
    
    public static Hood instance_ = null;
    public static boolean m_hoodDebug = true;

    //#region Constants
    //TODO Move these to Constants.java
    private static int HOOD_MOTOR_ID = 23;
    private static double kHoodMotorGearRatio = 81;
    private static double ticksPerDegree = 1/1;
    private static double degreesPerTick = 1/1;
    private static double kHoodP = 1.0D;
    private static double kHoodD = 0.0D;
    private static double kHoodFF = 0.005D;
    private static float kHoodUpperLimit = 50F;
    private static float kHoodLowerLimit = 0F;
    //#endregion

    private final CANSparkMax m_hood;
    private final SparkMaxPIDController m_hoodController;
    private final Hood.HoodDisplay m_hoodDisplay;

    private Hood() {
        this.setName("Hood");
        m_hood = new CANSparkMax(HOOD_MOTOR_ID, MotorType.kBrushless);
        m_hoodController = m_hood.getPIDController();
        m_hoodDisplay = this.new HoodDisplay();

        //#region Motor Config
        m_hood.restoreFactoryDefaults();
        m_hood.clearFaults();
        m_hood.setInverted(false);
        m_hood.setIdleMode(IdleMode.kBrake);
        m_hood.setSoftLimit(SoftLimitDirection.kForward, kHoodUpperLimit);
        m_hood.setSoftLimit(SoftLimitDirection.kReverse, kHoodLowerLimit);

        //PDF controller (no I coeff, probably not neccesary)
        m_hoodController.setP(kHoodP, 0);   
        m_hoodController.setD(kHoodD, 0);
        m_hoodController.setFF(kHoodFF, 0);
        //#endregion
                
        m_hood.getEncoder().setPosition(0);

    }

    @Override
    public void periodic() {
        if (m_hoodDebug) m_hoodDisplay.periodic();
        
        SmartDashboard.putNumber("HOod", m_hood.getEncoder().getVelocity());
    }

    public void zeroHood() {
        new RunCommand(() -> m_hoodController.setReference(-0.3, ControlType.kDutyCycle, 0), this)
        .withTimeout(1)
        .andThen(new InstantCommand(() -> m_hoodController.setReference(0, ControlType.kDutyCycle, 0), this))
        .andThen(new WaitCommand(1))
        .andThen(new InstantCommand(() -> {m_hood.getEncoder().setPosition(0);})).schedule();

    }

    //For manual hood adjustment
    private void setHoodSpeed(double s) {
        m_hood.set(s);
    }

    public double getHoodAngle() {
        return mapHoodAngle(m_hood.getEncoder().getPosition(), true);
    }

    public void setHoodAngle(double deg) {
        // m_hoodController.setReference(mapHoodAngle(deg, false), ControlType.kPosition); //TODO ADD MAPPING
        m_hoodController.setReference(deg, CANSparkMax.ControlType.kPosition);
    }

    private double getHoodMotorCurrent() {
        return m_hood.getOutputCurrent();
    }

    /**
     * @param convert true for converting ticks to degrees, false for converting degrees to ticks.
     * @return Ticks to degrees
     */
    private double mapHoodAngle(double value, boolean convertTicks) {
        final double kMinTicks = 0d;
        final double kMaxTicks = 0d; //TODO find with testing
        final double kMinDegrees = 53d;
        final double kMaxDegrees = 83d;
        return convertTicks ? MathUtil.clamp(value * ticksPerDegree, kMinTicks, kMaxTicks)
        : MathUtil.clamp(value * degreesPerTick, kMinDegrees, kMaxDegrees);
    }

    public static Hood getInstance() {
        if (instance_ == null) instance_ = new Hood();
        return instance_;
    }

    /**
     * @param speed neg speed for down, pos speed for up, 0 for stopping
     */
    public InstantCommand setSpeed(double speed) {
        return new InstantCommand(() -> this.setHoodSpeed(speed), this);
    }

    // private class HoodDisplay extends ShuffleboardDebugBoard {
    //     public HoodDisplay() {
    //         super("Debug", Hood.getInstance().getName(), 2, 7);
    //     }

    //     @Override
    //     protected void createObjects() {
            
    //     }   
    // }

    // Telemetry Class
    private class HoodDisplay {
        private ShuffleboardTab m_tab = Shuffleboard.getTab("Debug");
        private ShuffleboardLayout m_layout = m_tab.getLayout("Hood", BuiltInLayouts.kList)
        .withPosition(0, 0)
        .withSize(2, 6);
        private NetworkTableEntry m_current, m_posAngleEntry, m_posTicksEntry, m_setpointEntry, m_hoodPEntry, m_hoodDEntry, m_hoodFFEntry, m_setters;

        public HoodDisplay() {
            m_layout.add(Hood.this);

            m_current = m_layout.add("Current Amps", 0).withWidget(BuiltInWidgets.kDial).getEntry();
            m_posAngleEntry = m_layout.add("Position (Angle)", 0).withWidget(BuiltInWidgets.kTextView).getEntry();
            m_posTicksEntry = m_layout.add("Position (Ticks)", 0).withWidget(BuiltInWidgets.kTextView).getEntry();
            
            m_setpointEntry =  m_layout.add("Setpoint", 0d).withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("Min", 0, "Max", 60)).getEntry();
            m_hoodPEntry = m_layout.addPersistent("P", 1d).withWidget(BuiltInWidgets.kTextView).getEntry();
            m_hoodDEntry = m_layout.addPersistent("D", 0d).withWidget(BuiltInWidgets.kTextView).getEntry();
            m_hoodFFEntry = m_layout.addPersistent("FF", 0d).withWidget(BuiltInWidgets.kTextView).getEntry();
            m_layout.add(new InstantCommand(() -> Hood.this.zeroHood()));
            m_setters = m_layout.add("Use Debug Values", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
        }

        public void periodic() {

            m_current.setDouble(Hood.this.getHoodMotorCurrent());
            m_posAngleEntry.setDouble(Hood.this.getHoodAngle());
            m_posTicksEntry.setDouble(Hood.this.m_hood.getEncoder().getPosition());
            if (m_setters.getBoolean(false)) {
                setHoodAngle(m_setpointEntry.getDouble(0));
                m_hoodController.setP(m_hoodPEntry.getDouble(kHoodP), 0);   
                m_hoodController.setD(m_hoodDEntry.getDouble(kHoodD), 0);
                m_hoodController.setFF(m_hoodFFEntry.getDouble(kHoodFF), 0);
            }

        }
        
    }
}
