package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants;

public class DefaultShootCmd extends Command {
    private final ShooterSubsystem shooter;
    
    public DefaultShootCmd(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }
    
    public void execute() {
        shooter.setAngle(0.0);
        shooter.setShooter(Constants.SHOOTER_DEFAULT_RPS);
        shooter.setRoller(0.0);
        shooter.setMouth(0);
    }

    public boolean isFinished() {
        return false;
    }
}