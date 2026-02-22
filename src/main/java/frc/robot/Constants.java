package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Util.RectanglePoseArea;

public class Constants {

    public static double SHOOTER_DEFAULT_RPS = 33.33; // 2000 rpm
    public static double ROTATIONS_PER_INCH_CLIMBER = 0.0; // fix me
    public static double CLIMBER_MAX_HEIGHT = 8; // 8 in

    public static RectanglePoseArea BLUE_AREA = new RectanglePoseArea(new Translation2d(0.0, 0.0), new Translation2d(4.5, 8.0));
    public static RectanglePoseArea UPPER_BLUE_AREA = new RectanglePoseArea(new Translation2d(0.0, 4.0), new Translation2d(4.5, 8.0));
    public static RectanglePoseArea LOWER_BLUE_AREA = new RectanglePoseArea(new Translation2d(0.0, 0.0), new Translation2d(4.5, 4.0));
    public static RectanglePoseArea RED_AREA = new RectanglePoseArea(new Translation2d(12.0, 0.0), new Translation2d(16.5, 8.0));
    public static RectanglePoseArea UPPER_RED_AREA = new RectanglePoseArea(new Translation2d(12.0, 4.0), new Translation2d(16.5, 8.0));
    public static RectanglePoseArea LOWER_RED_AREA = new RectanglePoseArea(new Translation2d(12.0, 0.0), new Translation2d(16.5, 4.0));
    public static RectanglePoseArea UPPER_NEUTRAL = new RectanglePoseArea(new Translation2d(4.5, 4.0), new Translation2d(12.0, 8.0));
    public static RectanglePoseArea LOWER_NEUTRAL = new RectanglePoseArea(new Translation2d(4.5, 0.0), new Translation2d(12.0, 4.0));

    public static RectanglePoseArea BLUE_CLIMB_AREA = new RectanglePoseArea(new Translation2d(0.0, 3.0), new Translation2d(1.5, 4.5));
    public static RectanglePoseArea RED_CLIMB_AREA = new RectanglePoseArea(new Translation2d(15.0, 3.0), new Translation2d(16.5, 4.5));
}
