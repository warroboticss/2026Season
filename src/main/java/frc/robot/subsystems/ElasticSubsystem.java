package frc.robot.subsystems;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MatchConfig;

public class ElasticSubsystem extends SubsystemBase{
    private final NetworkTable elasticTable;
    private final DoublePublisher velocityPublisher;
    private final DoublePublisher voltagePublisher;
    private final BooleanPublisher activePublisher;
    private final DoublePublisher matchTimePublisher;

    private final MatchStateManagerSubsystem matchState;
    private final LimelightSubsystem vision;

    public ElasticSubsystem(MatchStateManagerSubsystem matchState, LimelightSubsystem vision) {
        this.matchState = matchState;
        this.vision = vision;

        elasticTable = NetworkTableInstance.getDefault().getTable("elastic_datatable");
        velocityPublisher = elasticTable.getDoubleTopic("Velocity").publish();
        voltagePublisher = elasticTable.getDoubleTopic("Voltage").publish();
        activePublisher = elasticTable.getBooleanTopic("State").publish();
        matchTimePublisher = elasticTable.getDoubleTopic("Match Time").publish();
    }


    public Command defaultElasticCmd() {
        return run(() -> {
            velocityPublisher.set(vision.getBotSpeed());
            voltagePublisher.set(RobotController.getBatteryVoltage());
            matchTimePublisher.set(Math.round(DriverStation.getMatchTime() * 10) / 10);
            if (MatchConfig.USE_MATCH_STATE) {
                activePublisher.set(matchState.getActive());
            }
        });
    }
}
