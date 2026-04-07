package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.Util.MatchData;

public class LimelightSubsystem extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;
    private final MatchData matchData;
    private SwerveDriveState driveState;

    private Boolean enable = true;
    private Boolean seeded = false;
    private String ll = "limelight-shooter";

    private int fieldError = 0;
    private int distanceError = 0;

    public LimelightSubsystem(CommandSwerveDrivetrain drivetrain, MatchData matchData) {
        this.matchData = matchData;
        this.drivetrain = drivetrain;
        SmartDashboard.putNumber("Field Error", fieldError);
        SmartDashboard.putNumber("Limelight Error", distanceError);
    }

    public Command defaultLLCmd() {
        return this.run(() -> {
            if (enable) {
            LimelightHelpers.SetRobotOrientation(ll, drivetrain.getState().Pose.getRotation().getDegrees(),0,0,0,0,0);
            if (!seeded) {
                Pose2d botPose = LimelightHelpers.getBotPose2d_wpiBlue(ll);
                if (LimelightHelpers.getTV(ll) && Constants.FIELD_AREA.isPoseWithinArea(botPose)) {
                    drivetrain.resetPose(botPose);
                    seeded = true;
                }
            }                
            if (LimelightHelpers.getTV(ll) && seeded) {
                Pose2d lastPose = drivetrain.getState().Pose;
                PoseEstimate mt2;
                if (LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(ll) != null) {
                    mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(ll);
                } else {
                    mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue(ll);
                }
                if (Constants.FIELD_AREA.isPoseWithinArea(mt2.pose) && (lastPose.getTranslation().getDistance(mt2.pose.getTranslation()) < 0.5)) {
                            drivetrain.addVisionMeasurement(
                                mt2.pose,
                                Timer.getFPGATimestamp() - ((LimelightHelpers.getLatency_Capture(ll) + LimelightHelpers.getLatency_Pipeline(ll)) / 1000.0),
                                VecBuilder.fill(0.5, 0.5, 9999999)); 
                } else {
                    fieldError++;
                    SmartDashboard.putNumber("Field Error", fieldError);
                }
            } else {
                distanceError++;
                SmartDashboard.putNumber("Distance Error", distanceError);
            }
        }
        driveState = drivetrain.getState();
        });
    }

    public void setSeeded(Boolean state) {
        this.seeded = state;
    }

    public double getAbsoluteDistanceFromTarget(Pose2d target) {
        Pose2d botPose = driveState.Pose;
        double distanceToTarget = target.getTranslation().getDistance(botPose.getTranslation());
        // returns the absolute linear distance to the target for our regressions
        return Math.abs(distanceToTarget);
    }

    public Pose2d getOffsetTarget(Pose2d target) {
        double distance = getAbsoluteDistanceFromTarget(target);
        double flightTime = -1 * 0.00656791 * Math.pow(distance,2) + 0.263805 * distance + 0.213562;

        // gets robot motion
        double vx_robotRelative = driveState.Speeds.vxMetersPerSecond;
        double vy_robotRelative = driveState.Speeds.vyMetersPerSecond;
        double robotRotation = driveState.Pose.getRotation().getRadians();

        // convert robot relative velocities to field relative velocities
        double vx_fieldRelative = vx_robotRelative * Math.cos(robotRotation) - Math.sin(robotRotation) * vy_robotRelative;
        double vy_fieldRelative = vx_robotRelative * Math.sin(robotRotation) + Math.cos(robotRotation) * vy_robotRelative;

        double driftX = (-1 * vx_fieldRelative * flightTime);
        double driftY = (-1 * vy_fieldRelative * flightTime);
        // returns offset target
        return new Pose2d(driftX + target.getX(), driftY + target.getY(), new Rotation2d(0)); 
    }
    
    public double getHeadingError(Pose2d target) {
        Pose2d botPose = driveState.Pose;
        double dist_x = target.getX() - botPose.getX();
        double dist_y = target.getY() - botPose.getY();

        double targetToNormal = Math.atan2(dist_y, dist_x) + Math.PI;
        double robotYaw = botPose.getRotation().getRadians();
        // returns angle to target (wrapped for π, -π)
        double angleError = targetToNormal - robotYaw;
        return Math.atan2(Math.sin(angleError), Math.cos(angleError));
    }

    public Pose2d getBotPose() {
        return driveState.Pose;
    }

    public double getBotSpeed() {
        double botVxMps = driveState.Speeds.vxMetersPerSecond;
        double botVyMps = driveState.Speeds.vyMetersPerSecond;
        return Math.sqrt(Math.pow(botVxMps, 2) + Math.pow(botVyMps, 2));
    }

    public Pose2d getTarget(){
        Pose2d botPose = driveState.Pose;
        Pose2d target = new Pose2d(0, 0, new Rotation2d(0));
        switch (matchData.ALLIANCE) {
            case "Blue":
                if (Constants.BLUE_AREA.isPoseWithinArea(botPose)) {
                    target = Constants.BLUE_HUB;
                } else if (Constants.UPPER_NEUTRAL.isPoseWithinArea(botPose)) {
                    target = new Pose2d(2.0, 6.0, new Rotation2d(0.0));
                } else if (Constants.LOWER_NEUTRAL.isPoseWithinArea(botPose)) {
                    target = new Pose2d(2.0, 2.0, new Rotation2d(0.0));
                }
                break;
            case "Red":
                if (Constants.RED_AREA.isPoseWithinArea(botPose)) {
                    target = Constants.RED_HUB;
                } else if (Constants.UPPER_NEUTRAL.isPoseWithinArea(botPose)) {
                    target = new Pose2d(14.5, 6.0, new Rotation2d(0.0));
                } else if (Constants.LOWER_NEUTRAL.isPoseWithinArea(botPose)) {
                    target = new Pose2d(14.5, 2.0, new Rotation2d(0.0));
                }
                break;
        }
        return target;
    }
}