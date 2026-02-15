 package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Limelight;

public class GetDownCmd extends Command {
    private Climber climber;
    private Limelight limelight;

    public GetDownCmd(Climber climber, Limelight limelight) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void execute(){
        climber.setClimber(Constants.CLIMBER_MAX_HEIGHT);
    }

    @Override
    public boolean isFinished(){
        if (!limelight.getClimbZone().isPoseWithinArea(limelight.getBotPose())) {
            return true;
        } else {
            return false;
        }
    }

    @Override 
    public void end(boolean interrupted){

    }
}
