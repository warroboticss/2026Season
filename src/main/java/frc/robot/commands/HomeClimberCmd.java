package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class HomeClimberCmd extends Command{
    private final Climber climber;

    public HomeClimberCmd(Climber climber){
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void execute(){
        climber.home();
    }
}