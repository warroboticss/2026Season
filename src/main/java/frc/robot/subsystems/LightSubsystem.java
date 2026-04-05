package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.MatchConfig;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;

public class LightSubsystem extends SubsystemBase {
    private final MatchStateManagerSubsystem matchState;

    private static final int kPort = 2;
    private static final int kLength = 22;

    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_buffer;

    public LightSubsystem(MatchStateManagerSubsystem matchState) {
        this.matchState = matchState;
        m_led = new AddressableLED(kPort);
        m_buffer = new AddressableLEDBuffer(kLength);

        m_led.setLength(kLength);
        m_led.setData(m_buffer);
        m_led.start();
    }

    @Override
    public void periodic() {
        m_led.setData(m_buffer);
    }

    public Command defaultLightCmd() {
        return this.run(() -> {
            if (DriverStation.isAutonomous()) {
                Constants.RAINBOW_SCROLL.applyTo(m_buffer);
            } else if (MatchConfig.USE_MATCH_STATE) {
                if (matchState.getActive()) {
                    Constants.SOLID_GREEN.applyTo(m_buffer);
                } else {
                    Constants.SOLID_RED.applyTo(m_buffer);
                }
            } else {
                MatchConfig.DEFAULT_PATTERN.applyTo(m_buffer);
            }
        });
    }
}