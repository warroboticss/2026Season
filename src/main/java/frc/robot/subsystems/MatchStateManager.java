package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.MatchData;

public class MatchStateManager extends SubsystemBase {
    private boolean active; // is our hub active?
    private MatchData matchData;
    private double startTime = 0.0;

    public MatchStateManager(MatchData matchData) {
        this.matchData = matchData;
        SmartDashboard.putNumber("Shift Time Remaining:", 0.0);
    }

    public boolean getActive() {
        return this.active;
    }

    @ Override
    public void periodic() {
        double timeRemaining = DriverStation.getMatchTime();
        double shiftNumber = 4 - Math.floor((timeRemaining - 30) / 25); 

        if (timeRemaining <= 30 || timeRemaining >= 130) {
            active = true;
        } else {
            if (shiftNumber % 2 == 1) { 
                active = !matchData.WE_WON_AUTO;
            } else {
                active = matchData.WE_WON_AUTO; 
            }
        }

        timeRemaining = Math.round(DriverStation.getMatchTime());
        if (timeRemaining <= 130 && timeRemaining > 30){
            if (timeRemaining == 130 || timeRemaining == 105 || timeRemaining == 80 || timeRemaining == 55) {
                startTime = Timer.getFPGATimestamp();
            }
            double currentTime = Timer.getFPGATimestamp();
            double shiftTime = Math.round((25 - (currentTime - startTime)) * 10.0) / 10.0;
            SmartDashboard.putNumber("Shift Time Remaining:", shiftTime);
        } else if (timeRemaining > 130) {
            if (timeRemaining == 140) {
                startTime = Timer.getFPGATimestamp();
            }
            double currentTime = Timer.getFPGATimestamp();
            double shiftTime = Math.round((10 - (currentTime - startTime)) * 10.0) / 10.0;
            SmartDashboard.putNumber("Shift Time Remaining:", shiftTime);
        }
    }

}
