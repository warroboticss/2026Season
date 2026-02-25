package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class DeployIntake extends Command {
    // Deploy then intake. Stop spin and retract on release.
    private final IntakeSubsystem intake;

    public DeployIntake(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    public void execute() {
        intake.setIntakePosition(7.4167); // dummy value
        intake.runIntake(0.8); // dummy value
    }

    public boolean isFinished(){
        return false;
    }

    public void end(boolean interrupted) {
        intake.stopIntake();
        //intake.setIntakePosition(0); // dummy value
    }
}