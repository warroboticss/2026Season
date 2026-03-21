package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class DeployIntake extends Command {
    private final IntakeSubsystem intake;

    public DeployIntake(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    public void execute() {
        intake.setIntakePosition(7.2);
        intake.runIntake(0.8);
    }

    public boolean isFinished(){
        return false;
    }

    public void end(boolean interrupted) {
        intake.stopIntake();
    }
}