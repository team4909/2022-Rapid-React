package frc.lib.bioniclib;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public abstract class ShuffleboardDebugBoard {
    
    private ArrayList<NetworkTableEntry> m_entries;
    private ShuffleboardTab m_tab;

    public ShuffleboardDebugBoard(String tabName, String layoutName, int w, int h) {
        m_tab = Shuffleboard.getTab("Debug");
    }

    protected abstract void createObjects();
}
