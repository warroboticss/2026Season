package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private final TalonFX climber = new TalonFX(19);
    private final MotionMagicVoltage angleRequest = new MotionMagicVoltage(0.0);
    private final DigitalInput home = new DigitalInput(0);
    public Boolean is_home = false;

    public Climber() {
       configMotor(climber.getConfigurator());     
       climber.optimizeBusUtilization();
    }


    // methods
    public boolean getHome() {
        return home.get();
    }

    public void home() {
        if (!home.get()) {
            climber.set(-0.3);
        } else {
            climber.set(0);
        }
    }

    public Command setClimber(double distance) {
        return this.run(() -> climber.setControl(angleRequest.withPosition(distance)));
    }

    // configs
    private static void configMotor(TalonFXConfigurator config) {
    // Creating a new configuration to ensure we get the same results every time
    var newConfig = new TalonFXConfiguration();

    // Configure idle mode and polarity
    var output = newConfig.MotorOutput;
    //output.Inverted = InvertedValue.Clockwise_Positive;
    output.NeutralMode = NeutralModeValue.Brake;

    // Set max voltage
    var voltage = newConfig.Voltage;
    voltage.PeakForwardVoltage = 12;
    voltage.PeakReverseVoltage = -12;

    // Set current limits
    var current = newConfig.CurrentLimits;
    current.StatorCurrentLimit = 120;
    current.StatorCurrentLimitEnable = true;
    current.SupplyCurrentLimit = 60;
    current.SupplyCurrentLimitEnable = true;

    // Configure PID in Slot 0
    Slot0Configs slot0 = newConfig.Slot0;
    slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

    slot0.kS = 0.25; 
    slot0.kV = 0.12;
    slot0.kA = 0.01;
    slot0.kP = 4.8;
    slot0.kI = 0;
    slot0.kD = 0.1; 
    
    // Configuring MotionMagic
    var motionMagic = newConfig.MotionMagic;
    motionMagic.MotionMagicCruiseVelocity = 80.0;
    motionMagic.MotionMagicAcceleration = 60.0;
    motionMagic.MotionMagicJerk = 1600.0;

    // Apply configuration
    config.apply(newConfig, 0.050);
  }
}