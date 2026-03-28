package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.MatchConfig;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class ShootCmd extends Command{
   
    private Shooter shooter;
    private Limelight vision;
    private IntakeSubsystem intake;

    private double shootSpeed;
    private double hoodRot;
    private Pose2d target;
    
    public ShootCmd(Shooter shooter, Limelight vision, IntakeSubsystem intake) {
        this.shooter = shooter;
        this.vision = vision;
        this.intake = intake;

        addRequirements(shooter);
    }
    
    public void execute() {
        shooter.setAngle(vision.getHoodAngle());
        shooter.setShooter(vision.getFlywheelRPS());
        if (MatchConfig.USE_OSCILATION) {
            intake.oscillateRoller();
        } else {
            intake.runIntake(1.0);
        }

        if(Math.abs(shooter.getHoodRotations() - hoodRot) < 0.2 && Math.abs(shooter.getShootSpeed() - shootSpeed) < 2 && vision.getHeadingError(target) <= 0.175){
            shooter.setRoller(-0.7);
            shooter.setMouth(0.9);
        }
    }

    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooter(Constants.SHOOTER_DEFAULT_RPS);
        shooter.setMouth(0.0);
        shooter.setRoller(-0.2);
        shooter.setAngle(0.0);
    }}