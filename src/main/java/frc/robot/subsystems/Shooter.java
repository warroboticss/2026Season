package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// all device ids are temporary and not canon

public class Shooter extends SubsystemBase{
    static final TalonFX rollersMain = new TalonFX(0);
    static final TalonFX rollerFollower = new TalonFX(1);
    static final TalonFX shooterMain = new TalonFX(2);
    static final TalonFX shooterFollower = new TalonFX(3);
    static final TalonFX hoodAngler = new TalonFX(4);
    static final TalonFX mouth = new TalonFX(5);
    final MotionMagicVoltage m_angleRequest = new MotionMagicVoltage(0.0);
    final MotionMagicVelocityVoltage m_shooterRequest = new MotionMagicVelocityVoltage(0.0);

    private double desiredShooterRPS = Constants.SHOOTER_DEFAULT_RPS;

    public Shooter(){
        rollerFollower.setControl(new Follower(0, MotorAlignmentValue.Aligned));
        shooterFollower.setControl(new Follower(0, MotorAlignmentValue.Aligned));

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

        shooterMain.getConfigurator().apply(talonFXConfigsShooter);

        var talonFXConfigsAngle = new TalonFXConfiguration();

        // tune this
        var angleConfigs = talonFXConfigsAngle.Slot0;
        angleConfigs.kS = 0.25; // Add 0.25 V output to overcome static friction
        angleConfigs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        angleConfigs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        angleConfigs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        angleConfigs.kI = 0; // no output for integrated error
        angleConfigs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic settings
        var motionMagicConfigsAngle = talonFXConfigsAngle.MotionMagic;
        motionMagicConfigsAngle.MotionMagicCruiseVelocity = 5;
        motionMagicConfigsAngle.MotionMagicAcceleration = 10;
        motionMagicConfigsAngle.MotionMagicJerk = 120;

        hoodAngler.getConfigurator().apply(talonFXConfigsAngle);
    }

    @Override
    public void periodic() {
        shooterMain.setControl(m_shooterRequest.withVelocity(desiredShooterRPS));
    }

    public void setRoller(double speed) {
        rollersMain.set(speed);
    }

    public void setShooter(double speed) {
        desiredShooterRPS = speed;
    }

    public void setAngle(double angle) {
        hoodAngler.setControl(m_angleRequest.withPosition(angle));
    }

    public void setMouth(double speed) {
        mouth.set(speed);
    }
}
