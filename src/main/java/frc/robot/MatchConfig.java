package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.LEDPattern;

public class MatchConfig {
    // Last Updated: 3/27/2026

    // INTAKE UP HEIGHT
    public static final double INTAKE_UP_POSITION = 0.0;
    // DEFAULT SHOOTER RPS
    public static final double DEFAULT_RPS = 20.0;
    // LOWER OSCILATION AMPLITUDE
    public static final double LOWER_AMP = 0.35;
    // UPPER OSCILATION AMPLITUDE
    public static final double UPPER_AMP = 0.8;
    // LIMELIGHTS TO USE
   public static final ArrayList<String> LIMELIGHTS = new ArrayList<String>(List.of("limelight-shooter"));
    // DEFAULT LED BEHAVIOR (IF MATCH STATE IS OFF)
    public static final LEDPattern DEFAULT_PATTERN = Constants.RAINBOW_SCROLL;
    /*
     Options:
        Constants.RAINBOW_SCROLL - Rainbow Gradient, Scrolling LED
        Constants.RED_SCROLL_GRADIENT - Red Gradient, Scrolling LED
        Constants.PINK_SCROLL_GRADIENT - Pink Gradient, Scrolling LED
        Constants.SOLID_GREEN - Solid Green Color
        Constants.SOLID_RED - Solid Red Color
        Constants.OFF - All Black
     */

    // USE MATCH STATE LED
    public static final boolean USE_MATCH_STATE = true;

    // DRIVER CONTROL SELECTION
    public static final double DRIVE_DEFAULT_SCALE = 0.35;
    public static final double DRIVE_SPRINT_SCALE = 0.7;
    public static final double DRIVE_SLOW_SCALE = 0.1;
    /*
     Options:
        0.25 - 25% of Applied Speed
        0.50 - 50% of Applied Speed
        0.75 - 75% of Applied Speed
        X.XX - XX% of Speed (you choose)
     */
}
