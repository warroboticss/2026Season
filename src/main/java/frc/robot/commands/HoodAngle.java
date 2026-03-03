package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants;
import frc.robot.Util.BotState;

public class HoodAngle extends Command {
    private final Shooter shooter;
    private double rotations;
    
    //public ShootOnTheMoveCmd(Shooter shooter, Limelight limelight, BotState botState) {
    public HoodAngle(Shooter shooter, double rotations) {
        this.shooter = shooter;
        this.rotations = rotations;
        addRequirements(shooter);
    }
    
    public void execute() {
        shooter.setAngle(rotations);
    }


    public boolean isFinished() {
            return Math.abs(shooter.getHoodRotations() - rotations) < 0.1;
        //return shooter.getHoodRotations() == rotations;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setAngle(0.0);
        shooter.setMouth(0.0);
        shooter.setRoller(0.0);
        shooter.setShooter(Constants.SHOOTER_DEFAULT_RPS);
    }
}
