package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class IntakeSubsystem extends SubsystemBase {
    // IDs not added yet, do not forget!
	private static TalonFX leftMotor = new TalonFX(0);
	private static TalonFX rightMotor = new TalonFX(0);
    private static TalonFX primaryIntakeMotor = new TalonFX(0);
    final static MotionMagicVoltage m_angleRequest = new MotionMagicVoltage(0.0);
	
    public IntakeSubsystem() {
        rightMotor.setControl(new Follower(0, MotorAlignmentValue.Aligned));

        var talonFXConfigsShooter = new TalonFXConfiguration();

        // tune this
        var shooterConfigs = talonFXConfigsShooter.Slot0;
        shooterConfigs.kS = 0.25; // Add 0.25 V output to overcome static friction
        shooterConfigs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        shooterConfigs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        shooterConfigs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        shooterConfigs.kI = 0; // no output for integrated error
        shooterConfigs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic settings
        var motionMagicConfigsShooter = talonFXConfigsShooter.MotionMagic;
        motionMagicConfigsShooter.MotionMagicCruiseVelocity = 80;
        motionMagicConfigsShooter.MotionMagicAcceleration = 160;
        motionMagicConfigsShooter.MotionMagicJerk = 1200;

        leftMotor.getConfigurator().apply(talonFXConfigsShooter);
    }

	public static void beginIntake(double speed){
	    primaryIntakeMotor.set(speed);
    }

    public static void ceaseAndDesistIntake(){
	    primaryIntakeMotor.set(0);
    }
	
	public static void setIntakePosition(double angle) {
		leftMotor.setControl(m_angleRequest.withPosition(angle));
    }
}
