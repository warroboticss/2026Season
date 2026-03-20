package frc.robot;

public class MatchConfig {
    // Last Updated: 3/18/2026

    // INTAKE UP HEIGHT
    public static final double INTAKE_UP_POSITION = 0.0;
    // DEFAULT SHOOTER RPS
    public static final double DEFAULT_RPS = 20.0;

    // AUTONOMOUS SELECTION
    public static final String AUTO = "Two Cycle";
    /*
     Options:
        "Two Cycle" - Intake/Shoot Twice, Ends in Alliance Zone
     */

    // DRIVER CONTROL SELECTION
    public static final double DRIVE_DEFAULT_SCALE = 0.25;
    public static final double DRIVE_SPRINT_SCALE = 0.7;
    /*
     Options:
        0.25 - 25% of Applied Speed
        0.50 - 50% of Applied Speed
        0.75 - 75% of Applied Speed
        X.XX - XX% of Speed (you choose)
     */
}
