package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    private final TalonFX climber = new TalonFX(0);
    private final MotionMagicVoltage angleRequest = new MotionMagicVoltage(0.0);
    private final DigitalInput home = new DigitalInput(0);
    public Boolean is_home = false;

    public Climber() {
        var talonFXConfigs = new TalonFXConfiguration();
        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        climber.getConfigurator().apply(talonFXConfigs);        
    }

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
        return this.run(() -> climber.setControl(angleRequest.withPosition(distance*Constants.ROTATIONS_PER_INCH_CLIMBER)));
    }

}