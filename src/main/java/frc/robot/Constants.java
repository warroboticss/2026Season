package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Util.RectanglePoseArea;

public class Constants {
    public static final double SHOOTER_DEFAULT_RPS = -20.0;
    public static final double CLIMB_ROT = 48.5;

    public static final Distance SPACING = Meters.of(1.0 / 60);
    public static final LEDPattern RAINBOW_SCROLL = LEDPattern.rainbow(255, 128).scrollAtAbsoluteSpeed(MetersPerSecond.of(1), SPACING);
    public static final LEDPattern RED_SCROLL_GRADIENT = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, new Color(0, 150, 0), new Color(0,255,0)).scrollAtAbsoluteSpeed(MetersPerSecond.of(1), SPACING);
    public static final LEDPattern PINK_SCROLL_GRADIENT = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, new Color(0, 255, 111), new Color(142, 237, 183)).scrollAtAbsoluteSpeed(MetersPerSecond.of(1), SPACING);
    public static final LEDPattern SOLID_BLUE = LEDPattern.solid(new Color(0, 0 , 220));
    public static final LEDPattern SOLID_GREEN = LEDPattern.solid(new Color(255, 0 , 0));
    public static final LEDPattern SOLID_RED = LEDPattern.solid(new Color(0, 255, 0));
    public static final LEDPattern OFF = LEDPattern.kOff;

    public static final RectanglePoseArea FIELD_AREA = new RectanglePoseArea(new Translation2d(0.0, 0.0), new Translation2d(16.54, 8.02)); 

    public static final RectanglePoseArea BLUE_AREA = new RectanglePoseArea(new Translation2d(0.0, 0.0), new Translation2d(4.5, 8.0));
    public static final RectanglePoseArea RED_AREA = new RectanglePoseArea(new Translation2d(12.0, 0.0), new Translation2d(16.5, 8.0));
    public static final RectanglePoseArea UPPER_NEUTRAL = new RectanglePoseArea(new Translation2d(4.5, 4.0), new Translation2d(12.0, 8.0));
    public static final RectanglePoseArea LOWER_NEUTRAL = new RectanglePoseArea(new Translation2d(4.5, 0.0), new Translation2d(12.0, 4.0));

    public static final Pose2d BLUE_HUB = new Pose2d(4.620, 4.030, new Rotation2d(0.0));
    public static final Pose2d RED_HUB = new Pose2d(11.930, 4.030, new Rotation2d(0.0));
}
