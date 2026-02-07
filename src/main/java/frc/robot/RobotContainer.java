// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElasticSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.MatchStateManager;


public class RobotContainer {
    public double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.FieldCentric driveTargeting = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(0.0) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric driveRobotOriented = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController controller = new CommandXboxController(0);

    public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    //private final ElasticSubsystem elasticSubsystem = new ElasticSubsystem();
    //private final Limelight vision = new Limelight(drivetrain);
    //private final MatchStateManager matchStateManager = new MatchStateManager();
    
    // private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem(drivetrain);
    private static  final MedianFilter txMedian = new MedianFilter(1);
    private static final LinearFilter txLowpass = LinearFilter.singlePoleIIR(0.1, 0.02); // 40 fps

    public static double lastValidTx = 0.0;
    private static long lastValidTimeMs = 0;
    private static final long txHoldMs = 75; // in milliseconds

    public boolean isFollowingPath = false;


    private final Trigger a = controller.a();
    private final Trigger b = controller.b();
    private final Trigger x = controller.x();
    private final Trigger y = controller.y();
    private final Trigger rightTrigger = controller.rightTrigger();

    private SendableChooser<Command> autoChooser;

    public RobotContainer() {
        isFollowingPath = false;
        autoChooser = AutoBuilder.buildAutoChooser("Example");
        configureBindings();
    }         


    // filters tx  values
    public static double getStableFilteredTX() {
        boolean tv = LimelightHelpers.getTV("limelight");
        if (tv) {
            double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
            double median = txMedian.calculate(tx);
            double filtered = txLowpass.calculate(median);
            lastValidTx = filtered;
            lastValidTimeMs = System.currentTimeMillis();
            return filtered;
        }

        if (System.currentTimeMillis() - lastValidTimeMs < txHoldMs) {
            return lastValidTx;
        }

        return Double.NaN;
    }
    

    // finds angle adjust based on robot velocity and filtered tx values
    public double TargetLockRotationDegrees() {
        double angle_adjust = 0;
        double tx = getStableFilteredTX();

       
        if (LimelightHelpers.getTV("limelight")){
            double kP_feedforward = 0.25;
            double kP_feedback = 4.2;
            double angleOffset = Math.toRadians(tx);
            double robot_velocity_x = drivetrain.getState().Speeds.vxMetersPerSecond;
            double robot_velocity_y = drivetrain.getState().Speeds.vyMetersPerSecond;
            double robot_lateral_velocity = -robot_velocity_x * Math.sin(angleOffset) + robot_velocity_y * Math.cos(angleOffset); 
            double feed_forward_adjust = (robot_lateral_velocity) * kP_feedforward;
            
            System.out.println(feed_forward_adjust);
            double feed_back_adjust = -1 * angleOffset * kP_feedback;
            angle_adjust = feed_back_adjust + (-1 * feed_forward_adjust);
        }
        
        return angle_adjust;
    }


    // uses pathplanner to generate path
    public Command GeneratePath() {
        // Load the path we want to pathfind to and follow
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile("WWI.I");
            // Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
            PathConstraints constraints = new PathConstraints(
                3.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720)
            );

            // Since AutoBuilder is configured, we can use it to build pathfinding commands
            Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                path,
                constraints
            );
            return pathfindingCommand;
        } catch (Exception error) {
            System.out.println("error :(");
            return Commands.none();
        } 
    }    

    
    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        a.onTrue(Commands.runOnce(()-> {
            if (!isFollowingPath) {
                isFollowingPath = true;
                Command limelightPath = GeneratePath();
                CommandScheduler.getInstance().schedule(limelightPath.andThen(() -> isFollowingPath = false));
            } // hello world! -- (Hunter, Kevin, Ahadu)
        }));

        // aligns to hub
        rightTrigger.whileTrue(drivetrain.applyRequest(() ->{
            return driveTargeting.withVelocityX((-controller.getLeftY() * MaxSpeed) * 0.5)
                                    .withVelocityY((-controller.getLeftX() * MaxSpeed) * 0.5)
                                    .withRotationalRate((TargetLockRotationDegrees()));
        } ));

        // reset the field-centric heading on left bumper press
        controller.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // resets isFollowingPath -> we should put this on elastic
        y.onTrue(new InstantCommand(() -> {
            isFollowingPath = false;
        }));


        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> {        
                if (!isFollowingPath) {
                return drive.withVelocityX((-controller.getLeftY() * MaxSpeed) * 0.5)
                                    .withVelocityY((-controller.getLeftX() * MaxSpeed) * 0.5)
                                    .withRotationalRate(-controller.getRightX() * MaxAngularRate);
            }
            else{
                return driveRobotOriented.withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0);
            }
        })); 

          

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // return autoChooser.getSelected();
        return Commands.none();
    }
}