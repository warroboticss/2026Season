package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ReverseHopperCmd extends Command{
   
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    
    public ReverseHopperCmd(ShooterSubsystem shooter, IntakeSubsystem intake) {
        this.shooter = shooter;
        this.intake = intake;

        addRequirements(shooter, intake);
    }
    
    public void execute() {
        shooter.setRoller(0.7);
        shooter.setMouth(-0.9);
        intake.runIntake(-0.8);
    }

    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted){
        intake.runIntake(0.0);
    }
}