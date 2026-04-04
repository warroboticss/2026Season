// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;

import frc.robot.commands.DeployIntakeCmd;
import frc.robot.commands.HomeClimberCmd;
import frc.robot.commands.LowerHoodCmd;
import frc.robot.commands.ReverseHopperCmd;
import frc.robot.commands.DefaultShootCmd;
import frc.robot.commands.ShootCmd;
import frc.robot.Util.MatchData;
import frc.robot.commands.AutoShootCmd;

import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElasticSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.MatchStateManagerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


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
    private final Telemetry logger = new Telemetry(MaxSpeed);

    // vars
    private double driveScale = MatchConfig.DRIVE_DEFAULT_SCALE;
    private SendableChooser<Command> autoChooser;

    // controller
    private final CommandXboxController controller = new CommandXboxController(0);
    private final Trigger a = controller.a();
    private final Trigger b = controller.b();
    private final Trigger y = controller.y();
    private final Trigger x = controller.x();
    private final Trigger rightTrigger = controller.rightTrigger();
    private final Trigger leftTrigger = controller.leftTrigger();
    private final Trigger leftBumper = controller.leftBumper();
    private final Trigger rightBumper = controller.rightBumper();

    // subsystems
    public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final MatchData matchData = new MatchData();
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final ClimberSubsystem climber = new ClimberSubsystem();
    public final LimelightSubsystem vision = new LimelightSubsystem(drivetrain, matchData);
    public final MatchStateManagerSubsystem stateManager = new MatchStateManagerSubsystem(matchData, controller);
    private final LightSubsystem light = new LightSubsystem(stateManager);
    private final ElasticSubsystem elastic = new ElasticSubsystem(stateManager, vision);

    // commands
    private final DeployIntakeCmd deployIntake = new DeployIntakeCmd(intake);
    private final InstantCommand intakeUp = new InstantCommand(() -> intake.setIntakePosition(MatchConfig.INTAKE_UP_POSITION));
    private final ReverseHopperCmd reverseHopperCmd = new ReverseHopperCmd(shooter, intake);
    private final InstantCommand sprintCmd = new InstantCommand(() -> setDriveScale(MatchConfig.DRIVE_SPRINT_SCALE));
    private final InstantCommand defaultScaleCmd = new InstantCommand(() -> { if (!a.getAsBoolean()){setDriveScale(MatchConfig.DRIVE_DEFAULT_SCALE);}});
    private final InstantCommand slowCmd = new InstantCommand(() -> setDriveScale(MatchConfig.DRIVE_SLOW_SCALE));
    private final InstantCommand seedVision = new InstantCommand(() -> vision.setSeeded(false));
    private final ParallelCommandGroup shootAndAlign = new ParallelCommandGroup(new ShootCmd(shooter, vision, intake), drivetrain.applyRequest(() -> {double error = vision.getHeadingError(vision.getTarget());return driveTargeting.withVelocityX((-controller.getLeftY() * MaxSpeed) * 0.2)
                                    .withVelocityY((-controller.getLeftX() * MaxSpeed) * 0.2)
                                    .withRotationalRate(Math.abs(9 * error) > 3.5 ? 3.5 * Math.signum(error) : 12 * error);}));

    //helper method
    public void setDriveScale(double scale) {
        driveScale = scale;
    }

    public RobotContainer() {
        NamedCommands.registerCommand("deployIntake", deployIntake);
        NamedCommands.registerCommand("shoot", new AutoShootCmd(shooter, intake).withTimeout(3.5));
        NamedCommands.registerCommand("lowerHood", new LowerHoodCmd(shooter));

        autoChooser = AutoBuilder.buildAutoChooser(Constants.AUTOS[0]);
        SmartDashboard.putData("Select Auto", autoChooser);

        shooter.setDefaultCommand(new DefaultShootCmd(shooter));
        climber.setDefaultCommand(new HomeClimberCmd(climber));
        light.setDefaultCommand(light.defaultLightCmd());
        elastic.setDefaultCommand(elastic.defaultElasticCmd());
        stateManager.setDefaultCommand(stateManager.defaultStateCmd());
        configureBindings();
    }    
    
    private void configureBindings() {
        x.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        y.whileTrue(intakeUp);
        a.whileTrue(climber.setClimber(Constants.CLIMB_ROT).alongWith(slowCmd));
        a.onFalse(defaultScaleCmd);
        b.onTrue(seedVision);

        leftTrigger.whileTrue(deployIntake);
        rightTrigger.whileTrue(shootAndAlign);
        rightTrigger.onTrue(seedVision);

        leftBumper.whileTrue(reverseHopperCmd);
        rightBumper.onTrue(sprintCmd);
        rightBumper.onFalse(defaultScaleCmd);

        // default drive
        drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> drive.withVelocityX((-controller.getLeftY() * MaxSpeed) * driveScale)
                                    .withVelocityY((-controller.getLeftX() * MaxSpeed) * driveScale)
                                    .withRotationalRate(-controller.getRightX() * MaxAngularRate)
        )); 

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        //return AutoBuilder.buildAuto(SmartDashboard.getData("Select Auto").toString()); 
        return Commands.none();
    }
}