package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;


public class LEDSubsystem extends SubsystemBase {
    private static final int PWMPort = 9;
    private static final int LEDLength = 20;

    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_buffer;

    public LEDSubsystem() { // the constructor
        m_led = new AddressableLED(PWMPort);
        m_buffer = new AddressableLEDBuffer(LEDLength);
        
        int activeBufferLength = 20; // how many LEDs in the team-colored "are we active?" section
        int scoringBufferLength = 20; // 
        AddressableLEDBufferView mBufferActive = m_buffer.createView(0, activeBufferLength - 1); // team colored; on when active, off otherwise
        AddressableLEDBufferView mBufferScoring = m_buffer.createView(activeBufferLength, activeBufferLength + scoringBufferLength - 1); // flashes when ready to score?
        
        m_led.setLength(LEDLength);
        m_led.start();
    }
    
    
    LEDPattern activePattern;
    LEDPattern scoringPattern;

    Color scoringColor = Color.kYellow;
    @Override
    public void periodic() {
        boolean active = MatchStateManager.getActive(); // are we active or not?

        if (active && Constants.ALLIANCE.contains("red")) { // active && DriverStation.getAlliance().toString().contains("red")
            activePattern = LEDPattern.solid(Color.kFirstRed);
        }  else if (active && Constants.ALLIANCE.contains("blue")) {
            activePattern = LEDPattern.solid(Color.kFirstBlue);
        } else {
            activePattern = LEDPattern.solid(Color.kDimGray);
        }

        if (Shooter.getShooting()) {
            // we want this to be flashing rapidly...hopefully not very illegal
            scoringPattern = LEDPattern.solid(scoringColor).blink(Time.ofBaseUnits(0.2, Seconds)).atBrightness(Dimensionless.ofBaseUnits(0.5, null));
        } else if (/* shooter isn't shooting, but error or precision or whatever is good for shooting */) {
            // slowly pulsing
            scoringPattern = LEDPattern.solid(scoringColor).breathe(Time.ofBaseUnits(2, Seconds)).atBrightness(Dimensionless.ofBaseUnits(0.5, null));
        } else if (/* while disabled, but not ready for shooting */) {
            scoringPattern = LEDPattern.solid(scoringColor);
        }

        
        m_led.setData(m_buffer);
    }

    public Command runPattern(LEDPattern pattern) {
        return run(() -> pattern.applyTo(m_buffer));
    }
}