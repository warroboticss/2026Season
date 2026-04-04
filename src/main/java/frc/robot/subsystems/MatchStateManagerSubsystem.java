package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Util.MatchData;

public class MatchStateManagerSubsystem extends SubsystemBase {
    private CommandXboxController controller;
    private boolean active; // is our hub active?
    private MatchData matchData;
    private double startTime = 0.0;

    public MatchStateManagerSubsystem(MatchData matchData, CommandXboxController controller) {
        this.matchData = matchData;
        this.controller = controller;
        SmartDashboard.putNumber("Shift Time Remaining", 0.0);
    }

    public boolean getActive() {
        return active;
    }

    public Command defaultStateCmd() {
        return run(() -> {
            double timeRemaining = Math.round(DriverStation.getMatchTime() * 10) / 10;
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

            if (timeRemaining <= 130 && timeRemaining > 30){ // Shift 1-4
                if (timeRemaining == 130 || timeRemaining == 105 || timeRemaining == 80 || timeRemaining == 55) {
                    startTime = Timer.getFPGATimestamp();
                }
                double shiftTime = Math.round((25 - (Timer.getFPGATimestamp() - startTime)) * 10.0) / 10.0;
                SmartDashboard.putNumber("Shift Time Remaining", shiftTime);
            } else if (timeRemaining > 130) { // Transition Period
                if (timeRemaining == 140) {
                    startTime = Timer.getFPGATimestamp();
                }
                double shiftTime = Math.round((10 - (Timer.getFPGATimestamp() - startTime)) * 10.0) / 10.0;
                SmartDashboard.putNumber("Shift Time Remaining", shiftTime);
            } else if (timeRemaining <= 30 && timeRemaining > 20) { // Endgame
                if (timeRemaining == 30) {
                    startTime = Timer.getFPGATimestamp();
                }
                double shiftTime = Math.round((30 - (Timer.getFPGATimestamp() - startTime)) * 10.0) / 10.0;
                SmartDashboard.putNumber("Shift Time Remaining", shiftTime);
            } else if (timeRemaining <= 20) { // Auto & Endgame
                if (timeRemaining == 20) {
                    startTime = Timer.getFPGATimestamp();
                }
                double shiftTime = Math.round((20 - (Timer.getFPGATimestamp() - startTime)) * 10.0) / 10.0;
                SmartDashboard.putNumber("Shift Time Remaining", shiftTime);
            }

            if ((timeRemaining <= 131 && timeRemaining > 130) || (timeRemaining <= 106 && timeRemaining > 105) || (timeRemaining <= 81 && timeRemaining > 80) || (timeRemaining <= 56 && timeRemaining > 55)) {
                controller.setRumble(RumbleType.kBothRumble, 0.5);
            } else {
                controller.setRumble(RumbleType.kBothRumble, 0.0);
            }
        });
    }

}
