package frc.robot;

import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

import frc.robot.subsystems.vision.Vision;

@RunWith(JUnit4.class)
public class LimelightTest implements AutoCloseable {

    Vision m_vision;
    
    @Before
    public void setup() {
        m_vision = Vision.getInstance();
    }
    
    @Test
    public void printDistance() {
        
    }

    @Override
    public void close() throws Exception {
        
    }
}