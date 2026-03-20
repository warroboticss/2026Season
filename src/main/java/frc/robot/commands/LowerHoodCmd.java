package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class LowerHoodCmd extends Command {
    private final Shooter shooter;
    
    public LowerHoodCmd(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }
    
    public void execute() {
        shooter.setAngle(0.0);
    }

    public boolean isFinished() {
        if (shooter.getHoodRotations() < 0.1) {
            return true;
        } else {
            return false;
        }
    }
}