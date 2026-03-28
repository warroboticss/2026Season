package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.MatchData;

public class MatchStateManager extends SubsystemBase {
    private boolean active; // is our hub active?
    private MatchData matchData;

    public MatchStateManager(MatchData matchData) {
        this.matchData = matchData;
    }

    public boolean getActive() {
        return this.active;
    }

    @ Override
    public void periodic() {
        double timeRemaining = DriverStation.getMatchTime();
        double shiftNumber = 4 - (Math.floor(timeRemaining - 30) / 25); 
        /* calculates which shift we're in
        for example: 90 secs before match end
        4 - (floor(90 - 30) / 25) = 2
        so 90 secs is in shift 2 */

        if (timeRemaining <= 30 || timeRemaining >= 130) {
            active = true;
        }
        else {
            if (shiftNumber % 2 == 1) { //  if shift number is odd; 1 or 3
                active = !matchData.WE_WON_AUTO; // will be active if we lost auto
            }
            else { // if shift number is even; 2 or 4
                active = matchData.WE_WON_AUTO; // will be active if we won auto
            }
        }
    }
}
