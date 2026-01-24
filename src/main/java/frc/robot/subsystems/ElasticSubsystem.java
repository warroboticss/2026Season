package frc.robot.subsystems;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

// The ElasticSubsystem periodically publishes custom data to our NetworkTable for Elastic

public class ElasticSubsystem extends SubsystemBase{
    private final NetworkTable elasticTable;
    // reports our filtered TX (after median and Low-Pass filters are applied)
    private final DoublePublisher m_filtered_tx;
    private final BooleanPublisher weWonAuto_Publisher;
    private final BooleanSubscriber weWonAuto_Subscriber;
    // private final BooleanPublisher matchState_Publisher;
    // private final BooleanSubscriber matchState_Subscriber;


    public ElasticSubsystem() {
        elasticTable = NetworkTableInstance.getDefault().getTable("elastic_datatable");
        m_filtered_tx = elasticTable.getDoubleTopic("TxFilterRd").publish();
        weWonAuto_Publisher = elasticTable.getBooleanTopic("weWonAuto").publish();
        weWonAuto_Subscriber = elasticTable.getBooleanTopic("weWonAuto").subscribe(true);

    }

    @Override
    public void periodic() {
        // periodically gets/sets data
        double filtered_tx = RobotContainer.getStableFilteredTX();
        m_filtered_tx.set(filtered_tx);
        weWonAuto_Publisher.set(weWonAuto_Subscriber.getAsBoolean());

    }
}
