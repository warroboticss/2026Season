package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.util.Color;


public class LEDSubsystem extends SubsystemBase {
    private static final int PWMPort = 9;
    private static final int LEDLength = 20;

    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_buffer;

    // public LEDSubsystem() {
        
    // }
    
    m_led = new AddressableLED(PWMPort);
    m_buffer = new AddressableLEDBuffer(LEDLength);
    AddressableLEDBufferView m_Buffer_active = m_buffer.createView(0, 10); // team colored; on when active, off otherwise
    AddressableLEDBufferView m_Buffer_scoring = m_buffer.createView(11, 20); // flashes when ready to score?
    m_led.setLength(LEDLength);
    m_led.start();
    LEDPattern pattern;
    // Set the default command to turn the strip off, otherwise the last colors written by
    // the last command to run will continue to be displayed.
    // Note: Other default patterns could be used instead!
    // setDefaultCommand(runPattern(LEDPattern.solid(Color.kBlack)).withName("Off"));
    

    @Override
    public void periodic() {
        boolean active = MatchStateManager.getActive(); // are we active or not?
        LEDPattern pattern;

        if (active && DriverStation.getAlliance().toString().contains("red")) {
                pattern = LEDPattern.solid(Color.kFirstRed);
            }  else if (active && DriverStation.getAlliance().toString().contains("blue")) {
                pattern = LEDPattern.solid(Color.kFirstBlue);
            } else {
                pattern = LEDPattern.solid(Color.kDimGray);
            }
        pattern.applyTo(m_Buffer_active);
        m_led.setData(m_buffer);
    }
    
}