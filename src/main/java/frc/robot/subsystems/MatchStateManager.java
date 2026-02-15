package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class MatchStateManager extends SubsystemBase {
    public boolean active; // is our hub active?
    boolean whoWonAuto; // get whoWonAuto from ElasticSubsystem

    public MatchStateManager() {
        
    }

    public boolean getActive() {
        return active;
    }

    @ Override
    public void periodic() {
    double timeRemaining = DriverStation.getMatchTime();
    double timeRemaining_beforeEndgame = timeRemaining - 30;
    //whoWonAuto = ElasticSubsystem.getWhoWonAuto();

        // System.out.println(active);
    }
}
