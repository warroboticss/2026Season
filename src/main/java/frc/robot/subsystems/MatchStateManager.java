package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class MatchStateManager extends SubsystemBase {

    @ Override
    public void periodic() {
    double timeremaining = DriverStation.getMatchTime();
    double timeremaining_beforeendgame = timeremaining - 30;
    boolean active;
    boolean weWonAuto = true;

        if (!(30 <= timeremaining && timeremaining <= 130)) { // if outside main 4 shifts
            active = true;
        }
        else {
            double shiftnumber = 4 - Math.floor(timeremaining_beforeendgame / 25); /* calculates which shift we're in
             * for example: 90 seconds before match end
             * 4 - floor(90/25) + 1 = 2
             * so 90 secs is in shift 2
             */
            if (shiftnumber % 2 == 1) { // if shift number is odd
                active = !weWonAuto; // will be on if we lost auto
            }
            else { // if shift number is even
                active = weWonAuto; // will be on if we won auto
            }
        }
        System.out.println(active);
        System.out.println(timeremaining);
    }
}
