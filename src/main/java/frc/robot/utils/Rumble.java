package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rumble extends SubsystemBase {

    private static Rumble m_instance = null;
    private XboxController[] m_controllers;

    // For creation of the rumble object
    private Rumble(XboxController... controllers) {
        m_controllers = controllers;
    }

    /**
     * @param rumbleStrength 0 for off, 1 for weak, 2 for medium, 3 for strong
     * @param duration the duration in seconds of the rumble. -1 will be an indefinite rumble
     * @param controllerIdx which controllers should rumble. [0] for driver, [1] for operator, [0, 1] for both
     * @return returns RunCommand MUST call .schedule for it to take effect.
     */
    public CommandGroupBase runRumble(int rumbleStrength, double duration, int... controllerIdx) {
        double[] strength = new double[2]; // [left, right]
        switch (rumbleStrength) {
            case 1: strength[0] = 1d; strength[1] = 0d; break;
            case 2: strength[0] = 0d; strength[1] = 1d; break;
            case 3: strength[0] = 1d; strength[1] = 1d; break;
            default: strength[0] = 0d; strength[1] = 0d; break;
        }
        return new RunCommand(() -> {
            for (int c : controllerIdx) {
                m_controllers[c].setRumble(GenericHID.RumbleType.kLeftRumble, strength[0]);
                m_controllers[c].setRumble(GenericHID.RumbleType.kRightRumble, strength[1]);
            }
        }, this).withTimeout(duration).andThen(
        new InstantCommand(() -> {
            for (int c : controllerIdx) {
                m_controllers[c].setRumble(GenericHID.RumbleType.kLeftRumble, 0);
                m_controllers[c].setRumble(GenericHID.RumbleType.kRightRumble, 0);
            }
        }, this));
    }

    public static Rumble getInstance() {
        return m_instance;
    }

    //Call this in robot Container first  or you will get a null pointer
    public static Rumble getInstance(XboxController... controllers) {
        if (m_instance == null) {
            m_instance = new Rumble(controllers);

        }
        return m_instance;
    }

}
