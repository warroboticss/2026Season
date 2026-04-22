package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class MatchStateManagerSubsystem extends SubsystemBase {
    private CommandXboxController controller;
    private double startTime = 0.0;

    public MatchStateManagerSubsystem(CommandXboxController controller) {
        this.controller = controller;
        SmartDashboard.putNumber("Shift Time Remaining", 0.0);
    }

    public boolean getActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (alliance.isEmpty()) {
            return false;
        }
        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }
        // At this point, if we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        // We're teleop enabled, compute.
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
        if (gameData.isEmpty()) {
            return true;
        }
        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
            // If we have invalid game data, assume hub is active.
            return true;
            }
        }

        // Shift was is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        if (matchTime > 130) {
            // Transition shift, hub is active.
            return true;
        } else if (matchTime > 105) {
            // Shift 1
            return shift1Active;
        } else if (matchTime > 80) {
            // Shift 2
            return !shift1Active;
        } else if (matchTime > 55) {
            // Shift 3
            return shift1Active;
        } else if (matchTime > 30) {
            // Shift 4
            return !shift1Active;
        } else {
            // End game, hub always active.
            return true;
        }
    }

    public Command defaultStateCmd() {
        return run(() -> {
            double timeRemaining = Math.round(DriverStation.getMatchTime() * 10) / 10;

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
