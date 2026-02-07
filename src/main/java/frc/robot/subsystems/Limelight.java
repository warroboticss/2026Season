package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
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
    // IMPORTANT COMMENT: Leo typed that ^^^
    CommandSwerveDrivetrain drivetrain;
    Alliance alliance;
    private String ll = "limelight";
    private Boolean enable = true;
    private Boolean trust = false;
    // error buildups number of errors
    public static int fieldError = 0;
    public static int distanceError = 0;
    private Pose2d botpose;
    private double confidence;
    private double targetDistance;
    // This creates our field so we ensure vision estimates are within the bodleif et fo sdn
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
}