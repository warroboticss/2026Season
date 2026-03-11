package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;

import frc.robot.Util.BotState;
import frc.robot.Util.RectanglePoseArea;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Limelight extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;
    private final BotState botState;
    private final List<String> limelights = new ArrayList<>(List.of("limelight-shooter"));
    private final Field2d fieldVisualization = new Field2d();

    private Boolean enable = true;
    private Boolean trust = false;
    public Boolean seeded = false;

    public int fieldError = 0;
    public int distanceError = 0;

    public Limelight(CommandSwerveDrivetrain drivetrain, BotState botState) {
        this.botState = botState;
        this.drivetrain = drivetrain;
        SmartDashboard.putNumber("Field Error", fieldError);
        SmartDashboard.putNumber("Limelight Error", distanceError);
        SmartDashboard.putData("Field", fieldVisualization);
    }

    @Override
    public void periodic() {
        if (enable) {
            for (String ll : limelights) {
                LimelightHelpers.SetRobotOrientation(ll, drivetrain.getState().Pose.getRotation().getDegrees(),0,0,0,0,0);
                Pose2d botPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight-shooter");
                if (!seeded && LimelightHelpers.getTV(ll) && Constants.FIELD_AREA.isPoseWithinArea(botPose)) {
                    drivetrain.resetPose(botPose);
                    seeded = true;
                }
                if (LimelightHelpers.getTV(ll) && seeded) {
                    Pose2d lastPose = drivetrain.getState().Pose;
                    PoseEstimate mt2;
                    if (LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(ll) != null) {
                        mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(ll);
                    } else{
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
                }
            }
        }
        fieldVisualization.setRobotPose(drivetrain.getState().Pose);
        fieldVisualization.getObject("target").setPose(getTarget());
        SmartDashboard.putData("Field", fieldVisualization);
        SmartDashboard.putNumber("Distance", getAbsoluteDistanceFromTarget(getTarget()));
       // SmartDashboard.putNumber("Distance: ", getAbsoluteDistanceFromTarget(new Pose2d(4.620, 4.030, new Rotation2d(0.0))));
    }

    public void useLimeLight(boolean enable){
        this.enable = enable;
    }

    public void trustLL(boolean trust){
        this.trust = trust;
    }

    public double getAbsoluteDistanceFromTarget(Pose2d target) {
        Pose2d botPose = drivetrain.getState().Pose;
        double distanceToTarget = target.getTranslation().getDistance(botPose.getTranslation());
        // returns the absolute linear distance to the target for our regressions
        return Math.abs(distanceToTarget);
    }

    public Translation2d getTargetOffset(double flightTime) {
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
    
    public double getHeadingError(Pose2d target) {
        Pose2d botPose = drivetrain.getState().Pose;
        double dist_x = target.getX() - botPose.getX();
        double dist_y = target.getY() - botPose.getY();

        double targetToNormal = Math.atan2(dist_y, dist_x) + Math.PI;
        double robotYaw = botPose.getRotation().getRadians();
        // returns angle to target (wrapped for π, -π)
        double angleError = targetToNormal - robotYaw;
        //System.out.println("Error: " + Math.atan2(Math.sin(angleError), Math.cos(angleError)));
        return Math.atan2(Math.sin(angleError), Math.cos(angleError));
    }

    public Pose2d getBotPose() {
        return drivetrain.getState().Pose;
    }

    public double getBotSpeed() {
        double botVxMps = drivetrain.getState().Speeds.vxMetersPerSecond;
        double botVyMps = drivetrain.getState().Speeds.vyMetersPerSecond;
        return Math.sqrt(Math.pow(botVxMps, 2) + Math.pow(botVyMps, 2));
    }

    public String getTrenchPath() {
        Pose2d botPose = drivetrain.getState().Pose;
        String trenchPath = null;
        switch (botState.ALLIANCE) {
            case "Blue":
                if (Constants.UPPER_BLUE_AREA.isPoseWithinArea(botPose)) {
                    trenchPath = "Upper Blue Path";
                } else if (Constants.LOWER_BLUE_AREA.isPoseWithinArea(botPose)) {
                    trenchPath = "Lower Blue Path";
                } else if (Constants.UPPER_NEUTRAL.isPoseWithinArea(botPose)) {
                    trenchPath = "Upper Neutral Blue Path";
                } else if (Constants.LOWER_NEUTRAL.isPoseWithinArea(botPose)) {
                    trenchPath = "Lower Neutral Blue Path";
                }
                break;
            case "Red":
                if (Constants.UPPER_RED_AREA.isPoseWithinArea(botPose)) {
                    trenchPath = "Upper Red Path";
                } else if (Constants.LOWER_RED_AREA.isPoseWithinArea(botPose)) {
                    trenchPath = "Lower Red Path";
                } else if (Constants.UPPER_NEUTRAL.isPoseWithinArea(botPose)) {
                    trenchPath = "Upper Neutral Red Path";
                } else if (Constants.LOWER_NEUTRAL.isPoseWithinArea(botPose)) {
                    trenchPath = "Lower Neutral Red Path";
                }
                break;
        }
        return trenchPath;
    }

    public String getClimbPath() {
        Pose2d botPose = drivetrain.getState().Pose;
        String climbPath = null;
        switch (botState.ALLIANCE) {
            case "Blue":
                if (Constants.UPPER_BLUE_AREA.isPoseWithinArea(botPose)) {
                    climbPath = "Upper Blue Climb";
                } else if (Constants.LOWER_BLUE_AREA.isPoseWithinArea(botPose)) {
                    climbPath = "Lower Blue Climb";
                }
                break;
            case "Red":
                if (Constants.UPPER_RED_AREA.isPoseWithinArea(botPose)) {
                    climbPath = "Upper Red Climb";
                } else if (Constants.LOWER_BLUE_AREA.isPoseWithinArea(botPose)) {
                    climbPath = "Lower Red Climb";
                }
                break;
        }
        return climbPath;
    }

    public RectanglePoseArea getClimbZone() {
        RectanglePoseArea poseArea = new RectanglePoseArea(new Translation2d(0,0), new Translation2d(0,0));
        switch (botState.ALLIANCE) {
            case "Blue":
                poseArea = Constants.BLUE_CLIMB_AREA;
                break;
            case "Red":
                poseArea = Constants.RED_CLIMB_AREA;
                break;
        }
        return poseArea;
    }

    public Pose2d getTarget(){
        Pose2d botPose = drivetrain.getState().Pose;
        Pose2d target = new Pose2d(0, 0, new Rotation2d(0));
        switch (botState.ALLIANCE) {
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



