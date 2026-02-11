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

public class ShootOnTheMove extends Command {
    private static Shooter shooter;
    private RectanglePoseArea blueArea = new RectanglePoseArea(new Translation2d(0.0, 0.0), new Translation2d(4.5, 8.0));
    private RectanglePoseArea redArea = new RectanglePoseArea(new Translation2d(12.0, 0.0), new Translation2d(16.5, 8.0));
    private RectanglePoseArea upperNeutral = new RectanglePoseArea(new Translation2d(4.5, 4.0), new Translation2d(12.0, 8.0));
    private RectanglePoseArea lowerNeutral = new RectanglePoseArea(new Translation2d(4.5, 0.0), new Translation2d(12.0, 4.0));

    
    public ShootOnTheMove(Shooter shooter) {
        ShootOnTheMove.shooter = shooter;
        addRequirements(shooter);
    }

    public void execute() {
        Pose2d botPose = Limelight.getBotPose();
        // first we pick our target
        Pose2d target = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
        switch (Constants.ALLIANCE) {
            case "Blue":
                if (blueArea.isPoseWithinArea(botPose)) {
                    target = new Pose2d(4.620, 4.030, new Rotation2d(0.0));
                } else if (upperNeutral.isPoseWithinArea(botPose)) {
                    target = new Pose2d(2.0, 6.0, new Rotation2d(0.0));
                } else if (lowerNeutral.isPoseWithinArea(botPose)) {
                    target = new Pose2d(2.0, 2.0, new Rotation2d(0.0));
                }
            case "Red":
                if (redArea.isPoseWithinArea(botPose)) {
                    target = new Pose2d(11.930, 4.030, new Rotation2d(0.0));
                } else if (upperNeutral.isPoseWithinArea(botPose)) {
                    target = new Pose2d(14.5, 6.0, new Rotation2d(0.0));
                } else if (lowerNeutral.isPoseWithinArea(botPose)) {
                    target = new Pose2d(14.5, 2.0, new Rotation2d(0.0));
                }
        }

        if (target.getX() != 0.0) {
            // checks if robot has a target we want to aim at
            double distance = Limelight.getAbsoluteDistanceFromTarget(target);
            double timeOfFlight = 0; //regression go here
            double angularVelocity = 0; //regression go here
            double hoodAngle = 0; //regression go here

            shooter.setAngle(hoodAngle);
            shooter.setShooter(angularVelocity);
        }
    }
}
