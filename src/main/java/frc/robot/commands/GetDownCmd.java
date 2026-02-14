 package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Limelight;

public class GetDownCmd extends Command {
    private Climber climber;

    public GetDownCmd(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void execute(){
        climber.setClimber(0);
    }

    @Override
    public boolean isFinished(){
        if (!Limelight.getClimbZone().isPoseWithinArea(Limelight.getBotPose())) {
            return true;
        } else {
            return false;
        }
    }

    @Override 
    public void end(boolean interrupted){

    }
}
