package frc.robot.subsystems.Extras;

import java.lang.ModuleLayer.Controller;
import java.time.Instant;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Rumble extends SubsystemBase {

    private static Rumble m_instance = null;
    private XboxController[] m_controllers;
    // private XboxController controller;
    // private boolean rumble;

    // FOr creation of the rumble object
    private Rumble(XboxController... controllers) {
        m_controllers = controllers;
    }

    public void rumbleON(){
        m_controllers[0].setRumble(GenericHID.RumbleType.kLeftRumble, 1.0);
        m_controllers[0].setRumble(GenericHID.RumbleType.kRightRumble, 1.0);
        // m_controllers[1].setRumble(GenericHID.RumbleType.kLeftRumble, 1.0);
        // m_controllers[1].setRumble(GenericHID.RumbleType.kRightRumble, 1.0);
        SmartDashboard.putString("rumble", "ON");
    }
    
    public void rumbleOFF(){
        m_controllers[0].setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
        m_controllers[0].setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
        // m_controllers[1].setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
        // m_controllers[1].setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
        SmartDashboard.putString("rumble", "OFF");
    }

    public CommandGroupBase runRumble() {
        SmartDashboard.putString("rumble", "CalledCommand");
        return new RunCommand(this::rumbleON, this)
        .withTimeout(1)
        .andThen(new InstantCommand(this::rumbleOFF, this));
    }

    public static Rumble getInstance() {
        return m_instance;
    }


    //Call this in robot Container first  or you will get a NULL POINTER
    public static Rumble getInstance(XboxController... controllers) {
        if (m_instance == null) {
            m_instance = new Rumble(controllers);

        }
        return m_instance;
    }
}
