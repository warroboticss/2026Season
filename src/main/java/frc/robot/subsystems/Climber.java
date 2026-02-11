package frc.robot.subsystems;
/* this was for path planner
import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
*/

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    static final TalonFX Climber = new TalonFX(0);
    final MotionMagicVoltage angleRequest = new MotionMagicVoltage(0.0);

    public Climber() {
        // in init function
        var talonFXConfigs = new TalonFXConfiguration();
        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        Climber.getConfigurator().apply(talonFXConfigs);        
    }

    // ??????????????????????????????????????????????????????????????????????????????????????
    public void setAngle(double angle) {
        Climber.setControl(angleRequest.withPosition(angle));
    }

    public void climb() {
        climber.setAngle(99999999);
    }

    // maybe more stuff to zero something blah blah blah?

    /* this is pp stuff but i guess its not well or something and it wont be used in ehre

   


    public PathPlannerPath createClimberPath() {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(""); // Placeholder until we make the path

            Pose2d currentPose = drive.getPose(); // idk i created this to get the position from odemetry??
            Optional<Pose2d> targetPose = path.getStartingHolonomicPose(); // From what I see, this gets the first position of the premade path?
            Pose2d targetGetPose = targetPose.get(); // I'm not sure if this is guarenteed to fetch the pose.

            // I just copied this from RobotContainer, I don't know what to do with such constraints
            PathConstraints constraints = new PathConstraints(
                3.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720)
            );

            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                currentPose,
                targetGetPose
            );
           
            PathPlannerPath generatedPath = new PathPlannerPath(waypoints, constraints, null, new GoalEndState(0.0, targetGetPose.getRotation()));

            return generatedPath;

         } catch (Exception error) {
            System.out.println("something wrong with creating path to tower for climbing");
            return null;
        }
    }*/
}
