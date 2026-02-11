package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Util.RectanglePoseArea;

public class ShootOnTheMoveCmd extends Command {
    private static Shooter shooter;
    public static Pose2d shooterTarget;
    
    public ShootOnTheMoveCmd(Shooter shooter) {
        ShootOnTheMoveCmd.shooter = shooter;
        addRequirements(shooter);
    }

    public void execute() {
        Pose2d botPose = Limelight.getBotPose();
        // picks our target
        Pose2d target = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
        switch (Constants.ALLIANCE) {
            case "Blue":
                if (Constants.BLUE_AREA.isPoseWithinArea(botPose)) {
                    target = new Pose2d(4.620, 4.030, new Rotation2d(0.0));
                } else if (Constants.UPPER_NEUTRAL.isPoseWithinArea(botPose)) {
                    target = new Pose2d(2.0, 6.0, new Rotation2d(0.0));
                } else if (Constants.LOWER_NEUTRAL.isPoseWithinArea(botPose)) {
                    target = new Pose2d(2.0, 2.0, new Rotation2d(0.0));
                }
                break;
            case "Red":
                if (Constants.RED_AREA.isPoseWithinArea(botPose)) {
                    target = new Pose2d(11.930, 4.030, new Rotation2d(0.0));
                } else if (Constants.UPPER_NEUTRAL.isPoseWithinArea(botPose)) {
                    target = new Pose2d(14.5, 6.0, new Rotation2d(0.0));
                } else if (Constants.LOWER_NEUTRAL.isPoseWithinArea(botPose)) {
                    target = new Pose2d(14.5, 2.0, new Rotation2d(0.0));
                }
                break;
        }

        if (target.getX() > 0.0) {
            double distance = Limelight.getAbsoluteDistanceFromTarget(target);
            double timeOfFlight = 0; //regression go here
            double angularVelocity = 0; //regression go here
            double hoodAngle = 0; //regression go here

            shooter.setAngle(hoodAngle);
            shooter.setShooter(angularVelocity);
            Translation2d targetOffset = Limelight.getTargetOffset(timeOfFlight);
            shooterTarget = new Pose2d(target.getX() + targetOffset.getX(), target.getY() + targetOffset.getY(), new Rotation2d(0.0));
            if (Math.abs(Limelight.getHeadingError(shooterTarget)) < 0.08) { // (if error is less than 5 deg) 
                shooter.setRoller(0.5); // pick number for this
                shooter.setMouth(0.5); // pick number for this
            }
        } else {
            shooterTarget = Limelight.getBotPose(); // aim nowhere
        }
    }

    public static double getTargetAngle() {
        return Limelight.getHeadingError(shooterTarget);
    }

    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setAngle(0.0);
        shooter.setMouth(0.0);
        shooter.setRoller(0.0);
        shooter.setShooter(Constants.SHOOTER_DEFAULT_RPS);
    }
}
