package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class IntakeSubsystem extends SubsystemBase {
	private final TalonFX leftMotor = new TalonFX(13);
	private final TalonFX rightMotor = new TalonFX(14);
    private final TalonFX primaryIntakeMotor = new TalonFX(17);
    private final MotionMagicVoltage m_angleRequest = new MotionMagicVoltage(0.0);
	
    private double intakePosition;

    public IntakeSubsystem() {
        rightMotor.setControl(new Follower(13, MotorAlignmentValue.Opposed));

        var talonFXConfigsIntake = new TalonFXConfiguration();

        // tune this
        var intakeConfigs = talonFXConfigsIntake.Slot0;

        intakeConfigs.kG = 0.34;
        intakeConfigs.kS = 0.25; // Add 0.25 V output to overcome static friction
        intakeConfigs.kV = 1.95; // A velocity target of 1 rps results in 0.12 V output
        intakeConfigs.kA = 0.06; // An acceleration of 1 rps/s requires 0.01 V output
        intakeConfigs.kP = 6;; // A position error of 2.5 rotations results in 12 V output
        intakeConfigs.kI = 0; // no output for integrated error
        intakeConfigs.kD = 0.25; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic settings
        var motionMagicConfigsIntake = talonFXConfigsIntake.MotionMagic;
        motionMagicConfigsIntake.MotionMagicCruiseVelocity = 80;
        motionMagicConfigsIntake.MotionMagicAcceleration = 160;
        motionMagicConfigsIntake.MotionMagicJerk = 1200;

        leftMotor.getConfigurator().apply(talonFXConfigsIntake);
    }

	public void runIntake(double speed){
	    primaryIntakeMotor.set(speed);
    }

    public void stopIntake(){
	    primaryIntakeMotor.set(0);
    }
	
	public void setIntakePosition(double angle) {
        leftMotor.setControl(m_angleRequest.withPosition(angle));
    }
}
