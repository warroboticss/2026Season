// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.DeployIntake;
import frc.robot.commands.HomeClimberCmd;
import frc.robot.commands.ShootOnTheMoveCmd;
import frc.robot.generated.TunerConstants;

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElasticSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.MatchStateManager;
import frc.robot.subsystems.Shooter;


public class RobotContainer {
    public double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.FieldCentric driveTargeting = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(0.0) // No deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric driveRobotOriented = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController controller = new CommandXboxController(0);
    private final Trigger a = controller.a();
    private final Trigger b = controller.b();
    private final Trigger x = controller.x();
    private final Trigger y = controller.y();
    private final Trigger rightTrigger = controller.rightTrigger();
    private final Trigger leftTrigger = controller.leftTrigger();

    public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Shooter shooter = new Shooter();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final Climber climber = new Climber();
    // private final ElasticSubsystem elasticSubsystem = new ElasticSubsystem();
    // private final Limelight vision = new Limelight(drivetrain);
    // private final MatchStateManager matchStateManager = new MatchStateManager();
    
    public boolean isFollowingPath = false;

    private SendableChooser<Command> autoChooser; 

    Command shootAndAlign = new ParallelCommandGroup(
        new RunCommand(() -> drivetrain.applyRequest(() -> { 
            double kP_feedback = 0.0; // tune this
            return driveTargeting.withVelocityX((-controller.getLeftY() * MaxSpeed) * 0.5)
                                    .withVelocityY((-controller.getLeftX() * MaxSpeed) * 0.5)
                                    .withRotationalRate(ShootOnTheMoveCmd.getTargetAngle() * kP_feedback);}
            ), drivetrain), new ShootOnTheMoveCmd(shooter)
    );

    public Command GeneratePath(String pathName) {
        try {  // Load the path we want to pathfind to and follow
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            PathConstraints constraints = new PathConstraints(3.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));
            Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(path, constraints);
            return pathfindingCommand;
        } catch (Exception error) {
            System.out.println("Error " + error);
            return Commands.none();
        } 
    }    

    public RobotContainer() {
        isFollowingPath = false;
        autoChooser = AutoBuilder.buildAutoChooser("Example");
        climber.setDefaultCommand(new HomeClimberCmd(climber));
        configureBindings();
    }    
    
    private void configureBindings() {
        a.onTrue(Commands.runOnce(()-> {
            if (!isFollowingPath) {
                isFollowingPath = true;
                String path = Limelight.getTrenchPath();
                if (path != null) {
                    Command limelightPath = GeneratePath(path);
                    CommandScheduler.getInstance().schedule(limelightPath.andThen(() -> isFollowingPath = false));
                }
            }
        }));

        rightTrigger.whileTrue(shootAndAlign);
        leftTrigger.whileTrue(new DeployIntake(intake));
        // reset the field-centric heading on left bumper press
        controller.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // default drive
        drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> {        
            if (!isFollowingPath) {
                return drive.withVelocityX((-controller.getLeftY() * MaxSpeed) * 0.7)
                                    .withVelocityY((-controller.getLeftX() * MaxSpeed) * 0.7)
                                    .withRotationalRate(-controller.getRightX() * MaxAngularRate);
            } else {
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
        // return autoChooser.getSelected(); // uncomment me when we have an auto
        return Commands.none();
    }
}