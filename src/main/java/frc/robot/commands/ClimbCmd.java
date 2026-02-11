package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers; // For pathplanner; will probably will need this position for the list of waypoints
import frc.robot.subsystems.Climber;

public class ClimbCmd extends Command {
    private static Climber climber;

    public void execute() {
        climber.climb();
    }

    public boolean isFinished() {
        return false; // 12345678910 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33
    }

    public void end() {
        climber.setAngle(0);
    }
}