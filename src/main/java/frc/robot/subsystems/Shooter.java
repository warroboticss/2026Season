package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// all device ids are temporary and not canon

public class Shooter extends SubsystemBase{
    private final TalonFX rollersMain = new TalonFX(16);
    private final TalonFX rollerFollower = new TalonFX(15);
    private final TalonFX shooterMain = new TalonFX(21);
    private final TalonFX shooterFollower = new TalonFX(20);
    private final TalonFX hoodAngler = new TalonFX(22);
    private final TalonFX mouth = new TalonFX(18);
    private final MotionMagicVoltage m_angleRequest = new MotionMagicVoltage(0.0);
    private final MotionMagicVelocityVoltage m_shooterRequest = new MotionMagicVelocityVoltage(0.0);

    private double desiredShooterRPS = Constants.SHOOTER_DEFAULT_RPS;
    public boolean shooting;

    public Shooter(){
        configShooterMotors(shooterMain.getConfigurator());
        configShooterMotors(shooterFollower.getConfigurator());
        shooterFollower.setControl(new Follower(21, MotorAlignmentValue.Opposed));

        configRollerMotors(rollersMain.getConfigurator());
        configRollerMotors(rollerFollower.getConfigurator());
        rollerFollower.setControl(new Follower(16, MotorAlignmentValue.Opposed));
        
        configAngleMotor(hoodAngler.getConfigurator());
        hoodAngler.setPosition(0);

        shooterMain.optimizeBusUtilization();
        shooterFollower.optimizeBusUtilization();
        rollersMain.optimizeBusUtilization();
        rollerFollower.optimizeBusUtilization();
        hoodAngler.optimizeBusUtilization();

    }

    // methods
    @Override
    public void periodic() {
        //shooterMain.setControl(m_shooterRequest.withVelocity(desiredShooterRPS));
        SmartDashboard.putNumber("hood angle", getHoodRotations());
    }

    public void setRoller(double speed) {
        rollersMain.set(speed);
    }

    public void setShooter(double speed) {
        shooterMain.setControl(m_shooterRequest.withVelocity(speed));
    }

    public void setAngle(double angle) {
        hoodAngler.setControl(m_angleRequest.withPosition(angle));
        System.out.println(angle);
    }

    public double getHoodRotations(){
        return hoodAngler.getPosition().getValueAsDouble();
    }

    public void setHoodPosition(){
        hoodAngler.setPosition(0);
    }

    public void setMouth(double speed) {
        mouth.set(speed);
    }

    public void setHood(double speed) {
        hoodAngler.set(speed);
    }

    public void setShooting(boolean state) {
        shooting = state;
    }

    public boolean getShooting() {
        return shooting;
    }


    // configs
    private static void configShooterMotors(TalonFXConfigurator config) {
    // Creating a new configuration to ensure we get the same results every time
    var newConfig = new TalonFXConfiguration();

    // Configure idle mode and polarity
    var output = newConfig.MotorOutput;
    output.Inverted = InvertedValue.Clockwise_Positive;
    output.NeutralMode = NeutralModeValue.Coast;

    // Set max voltage
    var voltage = newConfig.Voltage;
    voltage.PeakForwardVoltage = 14;
    voltage.PeakReverseVoltage = 0;

    // Set current limits
    var current = newConfig.CurrentLimits;
    current.StatorCurrentLimit = 100;
    current.StatorCurrentLimitEnable = true;
    current.SupplyCurrentLimit = 50;
    current.SupplyCurrentLimitEnable = true;

    // Configure PID in Slot 0
    Slot0Configs slot0 = newConfig.Slot0;
    slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

    slot0.kS = 0.21678;
    slot0.kV = 0.12259; 
    slot0.kA = 0.005602;
    slot0.kP = 0; 
    slot0.kI = 0;
    slot0.kD = 0; 
    
    // Configuring MotionMagic
    var motionMagic = newConfig.MotionMagic;
    motionMagic.MotionMagicCruiseVelocity = 80.0;
    motionMagic.MotionMagicAcceleration = 160.0;
    motionMagic.MotionMagicJerk = 1200.0;

    // Apply configuration
    config.apply(newConfig, 0.050);
  }

  private static void configAngleMotor(TalonFXConfigurator config) {
    // Creating a new configuration to ensure we get the same results every time
    var newConfig = new TalonFXConfiguration();

    // Configure idle mode and polarity
    var output = newConfig.MotorOutput;
    output.Inverted = InvertedValue.Clockwise_Positive;
    output.NeutralMode = NeutralModeValue.Brake;

    // Set max voltage
    var voltage = newConfig.Voltage;
    voltage.PeakForwardVoltage = 6;
    voltage.PeakReverseVoltage = -6;

    // Set current limits
    var current = newConfig.CurrentLimits;
    current.StatorCurrentLimit = 60;
    current.StatorCurrentLimitEnable = true;
    current.SupplyCurrentLimit = 30;
    current.SupplyCurrentLimitEnable = true;

    // Configure PID in Slot 0
    Slot0Configs slot0 = newConfig.Slot0;
    slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

    slot0.kS = 0.6; 
    slot0.kV = 0.05; 
    slot0.kA = 0.01; 
    slot0.kP = 0.4; 
    slot0.kI = 0; 
    slot0.kD = 0.1;
    
    // Configuring MotionMagic
    var motionMagic = newConfig.MotionMagic;
    motionMagic.MotionMagicCruiseVelocity = 5.0;
    motionMagic.MotionMagicAcceleration = 10.0;
    motionMagic.MotionMagicJerk = 100.0;

    // Apply configuration
    config.apply(newConfig, 0.050);
  }

  private static void configRollerMotors(TalonFXConfigurator config){
    var newConfig = new TalonFXConfiguration();

    // Configure idle mode and polarity
    var output = newConfig.MotorOutput;
    output.Inverted = InvertedValue.Clockwise_Positive;
    output.NeutralMode = NeutralModeValue.Coast;

    // Set max voltage
    var voltage = newConfig.Voltage;
    voltage.PeakForwardVoltage = 8;
    voltage.PeakReverseVoltage = -8;

    // Set current limits
    var current = newConfig.CurrentLimits;
    current.StatorCurrentLimit = 80;
    current.StatorCurrentLimitEnable = true;
    current.SupplyCurrentLimit = 40;
    current.SupplyCurrentLimitEnable = true;

    // Apply configuration
    config.apply(newConfig, 0.050);
  }
}
