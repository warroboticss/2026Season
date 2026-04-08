package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShootCmd extends Command{
   
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;

    private double shootSpeed;
    private double hoodRot;
    private double distance;
    private double kPVelDamp = 0.175;
    
    public AutoShootCmd(ShooterSubsystem shooter, IntakeSubsystem intake) {
        this.shooter = shooter;
        this.intake = intake;

        addRequirements(shooter);
    }
    
    public void execute() {
        distance = 2.0;
        shootSpeed = -1 * 6.39816 * distance - 33.10835;
        hoodRot = 0.641169 + 1.12764 * Math.log(distance);

        //hood checks
        if (hoodRot <= 0) {
            hoodRot = 0;
        } else if (hoodRot > 2.45) {
            hoodRot = 2.45;
        }

        // shoot checks
        if (Math.signum(shootSpeed) == 1) {
            shootSpeed = Constants.SHOOTER_DEFAULT_RPS;
        } else if (distance >= 4.0) {
            shootSpeed += distance * kPVelDamp;
        }

        shooter.setAngle(hoodRot);
        shooter.setShooter(shootSpeed);
        intake.oscillateRoller();

        if(Math.abs(shooter.getHoodRotations() - hoodRot) < 0.2 && Math.abs(shooter.getShootSpeed() - shootSpeed) < 2){
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
        intake.runIntake(1.0);
    }}