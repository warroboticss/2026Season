//help me help me help me help e help mee help mm
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class DeployIntake extends Command {
    private static IntakeSubsystem intake;

    public DeployIntake(IntakeSubsystem intake) {
        DeployIntake.intake = intake;
        addRequirements(intake);
    }

    // While ltrigger held- Deploy then feeding. Stop spin and retract on release.
    public void execute() {
        IntakeSubsystem.setIntakePosition(999999);
        IntakeSubsystem.beginIntake(5);
    }

    public boolean isFinished(){
        return false;
    }

    public void end() {
        IntakeSubsystem.ceaseAndDesistIntake();
        IntakeSubsystem.setIntakePosition(99999);
    }
}