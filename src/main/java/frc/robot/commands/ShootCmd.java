package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class ShootCmd extends Command{
   
    private Shooter shooter;
    private Limelight vision;
    private IntakeSubsystem intake;

    private double shootSpeed;
    private double hoodRot;
    
    public ShootCmd(Shooter shooter, Limelight vision, IntakeSubsystem intake) {
        this.shooter = shooter;
        this.vision = vision;
        this.intake = intake;

        addRequirements(shooter, intake);
    }
    
    public void execute() {
        shootSpeed = -1 * 6.39816 * vision.getAbsoluteDistanceFromTarget(vision.getTarget()) - 33.10835;
        hoodRot = 0.641169 + 1.12764 * Math.log(vision.getAbsoluteDistanceFromTarget(vision.getTarget()));
        shooter.setAngle(hoodRot);
        shooter.setShooter(shootSpeed);
        intake.runIntake(0.8);
        //intake.oscillateRoller();

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
        intake.runIntake(0);
    }}