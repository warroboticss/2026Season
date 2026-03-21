package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class ShootCmd extends Command{
   
    private Shooter shooter;
    private Limelight vision;
    private IntakeSubsystem intake;

    private double shootSpeed;
    private double hoodRot;
    private double distance;
    private Pose2d target;
    private double kPVelDamp = 0.175;
    
    public ShootCmd(Shooter shooter, Limelight vision, IntakeSubsystem intake) {
        this.shooter = shooter;
        this.vision = vision;
        this.intake = intake;

        addRequirements(shooter);
    }
    
    public void execute() {
        target = vision.getTarget();
        distance = vision.getAbsoluteDistanceFromTarget(target);
        shootSpeed = -1 * 6.39816 * distance - 33.10835 + 1;
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
        //intake.oscillateRoller();
        intake.runIntake(1.0);

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