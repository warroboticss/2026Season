package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class HomeClimberCmd extends Command{
    // this command makes the climber home automatically
    private final Climber climber;

    public HomeClimberCmd(Climber climber){
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void execute(){
        if (!climber.getKeepClimbing()) {
            climber.home();
        }
    }

    @Override
    public boolean isFinished(){
        return climber.getHome();   
    }
}