package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Util.RectanglePoseArea;

public class Constants {
    public static double SHOOTER_DEFAULT_RPS = -1 * Math.abs(MatchConfig.DEFAULT_RPS);
    public static double CLIMB_ROT = 47.5;

    public static RectanglePoseArea FIELD_AREA = new RectanglePoseArea(new Translation2d(0.0, 0.0), new Translation2d(16.54, 8.02)); 

    public static RectanglePoseArea BLUE_AREA = new RectanglePoseArea(new Translation2d(0.0, 0.0), new Translation2d(4.5, 8.0));
    public static RectanglePoseArea RED_AREA = new RectanglePoseArea(new Translation2d(12.0, 0.0), new Translation2d(16.5, 8.0));
    public static RectanglePoseArea UPPER_NEUTRAL = new RectanglePoseArea(new Translation2d(4.5, 4.0), new Translation2d(12.0, 8.0));
    public static RectanglePoseArea LOWER_NEUTRAL = new RectanglePoseArea(new Translation2d(4.5, 0.0), new Translation2d(12.0, 4.0));

    public static Pose2d BLUE_HUB = new Pose2d(4.620, 4.030, new Rotation2d(0.0));
    public static Pose2d RED_HUB = new Pose2d(11.930, 4.030, new Rotation2d(0.0));
}
