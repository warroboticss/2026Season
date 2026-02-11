package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.RectanglePoseArea;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {
    // defines all variables used for vison pose estimmates
    static CommandSwerveDrivetrain drivetrain;
    Alliance alliance;
    private static String ll = "limelight";
    private Boolean enable = true;
    private Boolean trust = false;
    // error buildups (number of errors)
    public static int fieldError = 0;
    public static int distanceError = 0;
    private Pose2d botpose;
    private double confidence;
    private double targetDistance;
    // This creates our field so we ensure vision estimates are within its bounds
    private static final RectanglePoseArea field = 
        new RectanglePoseArea(new Translation2d(0.0, 0.0), new Translation2d(16.54, 8.02));
    public Limelight(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        SmartDashboard.putNumber("Field Error", fieldError);
        SmartDashboard.putNumber("Limelight Error", distanceError);
    }

    @Override
    public void periodic() {
        if (enable) {
            targetDistance = LimelightHelpers.getTargetPose3d_CameraSpace(ll).getTranslation().getDistance(new Translation3d());
            confidence = 1 - ((targetDistance - 1) / 6);
            if (LimelightHelpers.getTV(ll)) {
                botpose = LimelightHelpers.getBotPose2d_wpiBlue(ll);
                if (field.isPoseWithinArea(botpose)) {
                    if (drivetrain.getState().Pose.getTranslation().getDistance(botpose.getTranslation()) < 0.5
                        || trust
                        || LimelightHelpers.getRawFiducials(ll).length > 1) {
                            drivetrain.addVisionMeasurement(
                                botpose,
                                Timer.getFPGATimestamp() - LimelightHelpers.getLatency_Capture("limelightName") - LimelightHelpers.getLatency_Pipeline("limelightName"),
                                VecBuilder.fill(confidence, confidence, .01));
                        } else {
                            distanceError++;
                            SmartDashboard.putNumber("Limelight Error", distanceError);
                        }
                } else {
                    fieldError++;
                    SmartDashboard.putNumber("Field Error", fieldError);
                }
            }
        }
    }

    public void setAlliance(Alliance alliance){
        this.alliance = alliance;
    }

    public void useLimeLight(boolean enable){
        this.enable = enable;
    }

    public void trustLL(boolean trust){
        this.trust = trust;
    }

    public static double getAbsoluteDistanceFromTarget(Pose2d target) {
        Pose2d botPose = drivetrain.getState().Pose;
        double distanceToTarget = target.getTranslation().getDistance(botPose.getTranslation());
        // returns the absolute linear distance to the target for our regressions
        return Math.abs(distanceToTarget);
    }

    public static Translation2d getTargetOffset(double flightTime) {
        // gets robot motion
        double vx_robotRelative = drivetrain.getState().Speeds.vxMetersPerSecond;
        double vy_robotRelative = drivetrain.getState().Speeds.vyMetersPerSecond;
        double robotRotation = drivetrain.getState().Pose.getRotation().getRadians();

        // convert robot relative velocities to field relative velocities
        double vx_fieldRelative = vx_robotRelative * Math.cos(robotRotation) - Math.sin(robotRotation) * vy_robotRelative;
        double vy_fieldRelative = vx_robotRelative * Math.sin(robotRotation) + Math.cos(robotRotation) * vy_robotRelative;
        // returns target offset (inverted)
        return new Translation2d(-vx_fieldRelative * flightTime, -vy_fieldRelative * flightTime); 
    }
    
    public static double getHeadingError(Pose2d target) {
        Pose2d botPose = drivetrain.getState().Pose;
        double dist_x = target.getX() - botPose.getX();
        double dist_y = target.getY() - botPose.getY();

        Rotation2d targetToNormal = new Rotation2d(Math.atan2(dist_y, dist_x));
        Rotation2d robotYaw = botPose.getRotation();
        // returns angle to target (wrapped for π, -π)
        double angleError = targetToNormal.minus(robotYaw).getRadians();
        return angleError;
    }

    public static Pose2d getBotPose() {
        return drivetrain.getState().Pose;
    }
}