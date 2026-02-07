package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.util.Color;

public class LEDSubsystem extends SubsystemBase {
    AddressableLED m_led = new AddressableLED(9);
    boolean active = MatchStateManager.getActive(); // are we active or not?
    LEDPattern pattern;
    AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(60);
    
    // Set the data
    public void periodic() {
        
        if (active && DriverStation.getAlliance().toString().contains("red")) {
            pattern = LEDPattern.solid(Color.kRed);
        }  else if (active && DriverStation.getAlliance().toString().contains("blue")) {
            pattern = LEDPattern.solid(Color.kBlue);
        } else {
            pattern = LEDPattern.solid(null);
        }
        m_led.setLength(m_ledBuffer.getLength());
        pattern.applyTo(m_ledBuffer);
        m_led.setData(m_ledBuffer);
        m_led.start();
    }
}