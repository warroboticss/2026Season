package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class DeployIntakeCmd extends Command {
    private final IntakeSubsystem intake;

    public DeployIntakeCmd(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    public void execute() {
        intake.setIntakePosition(7.2);
        intake.setOverrideOscilation(true);
        intake.runIntake(1.0);
    }

    public boolean isFinished(){
        return false;
    }

    public void end(boolean interrupted) {
        intake.setOverrideOscilation(false);
        intake.stopIntake();
    }
}