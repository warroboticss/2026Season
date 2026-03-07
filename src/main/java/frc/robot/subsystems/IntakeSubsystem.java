package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class IntakeSubsystem extends SubsystemBase {
	private final TalonFX leftMotor = new TalonFX(13);
	private final TalonFX rightMotor = new TalonFX(14);
    private final TalonFX primaryIntakeMotor = new TalonFX(17);
    private final MotionMagicVoltage m_angleRequest = new MotionMagicVoltage(0.0);
	
    private double intakePosition;

    public IntakeSubsystem() {
        configDeployMotors(leftMotor.getConfigurator());
        configDeployMotors(rightMotor.getConfigurator());
        //rightMotor.setControl(new Follower(13, MotorAlignmentValue.Opposed));
        
        configSpinnyMotor(primaryIntakeMotor.getConfigurator());

        leftMotor.optimizeBusUtilization();
        rightMotor.optimizeBusUtilization();
        primaryIntakeMotor.optimizeBusUtilization();
    }

    // methods
	public void runIntake(double speed){
	    primaryIntakeMotor.set(speed);
    }

    public void stopIntake(){
	    primaryIntakeMotor.set(0);
    }
	
	public void setIntakePosition(double angle) {
        leftMotor.setControl(m_angleRequest.withPosition(angle));
        rightMotor.setControl(m_angleRequest.withPosition(-angle));
    }
  
  public void setDeploySpeed(double speed){
    leftMotor.set(0);
  }


    // configs
    private static void configDeployMotors(TalonFXConfigurator config) {
    // Creating a new configuration to ensure we get the same results every time
    var newConfig = new TalonFXConfiguration();

    // Configure idle mode and polarity
    var output = newConfig.MotorOutput;
    //output.Inverted = InvertedValue.Clockwise_Positive;
    output.NeutralMode = NeutralModeValue.Brake;

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

    // Configure PID in Slot 0
    Slot0Configs slot0 = newConfig.Slot0;
    slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

    slot0.kG = 0.38;
    slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0.kV = 1.95; // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = 0.06; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = 6.27; // A position error of 2.5 rotations results in 12 V output
    slot0.kI = 0; // no output for integrated error
    slot0.kD = 0.25; // A velocity error of 1 rps results in 0.1 V output
    
    // Configuring MotionMagic
    var motionMagic = newConfig.MotionMagic;
    motionMagic.MotionMagicCruiseVelocity = 20.0;
    motionMagic.MotionMagicAcceleration = 40.0;
    motionMagic.MotionMagicJerk = 400.0;

    // Apply configuration
    config.apply(newConfig, 0.050);
  }

  private static void configSpinnyMotor(TalonFXConfigurator config){
    var newConfig = new TalonFXConfiguration();

    // Configure idle mode and polarity
    var output = newConfig.MotorOutput;
    //output.Inverted = InvertedValue.Clockwise_Positive;
    output.NeutralMode = NeutralModeValue.Coast;

    // Set max voltage
    var voltage = newConfig.Voltage;
    voltage.PeakForwardVoltage = 4;
    voltage.PeakReverseVoltage = -4;

    // Set current limits
    var current = newConfig.CurrentLimits;
    current.StatorCurrentLimit = 60;
    current.StatorCurrentLimitEnable = true;
    current.SupplyCurrentLimit = 30;
    current.SupplyCurrentLimitEnable = true;

    // Apply configuration
    config.apply(newConfig, 0.050);
  }

}
