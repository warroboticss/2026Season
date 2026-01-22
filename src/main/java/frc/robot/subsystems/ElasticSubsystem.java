package frc.robot.subsystems;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;

// The ElasticSubsystem periodically publishes custom data to our NetworkTable for Elastic

public class ElasticSubsystem extends SubsystemBase{
    private final NetworkTable elasticTable;
    private final DoublePublisher m_taPub;
    private final DoublePublisher m_txPub;
    private final DoublePublisher m_tyPub;
    private final DoublePublisher m_kpfeedfoward;
    private final DoublePublisher m_kpfeedback;
    private final DoublePublisher m_filter_tx;
    private final DoubleSubscriber feedforwardsubscriber;
    private final DoubleSubscriber feedbacksubscriber; 
    private final BooleanSubscriber subscriber_idkman_feedforwardsubscriber;
    private final BooleanPublisher subscriber_idkman_feedforwardsubscriber_publisher;
    public static double feedback;
    public static double feedforward;
    public static boolean Usefeedforward;



    public ElasticSubsystem() {
        elasticTable = NetworkTableInstance.getDefault().getTable("elastic_datatable");
        m_taPub = elasticTable.getDoubleTopic("ta").publish();
        m_txPub = elasticTable.getDoubleTopic("tx").publish();
        m_tyPub = elasticTable.getDoubleTopic("ty").publish();

        m_kpfeedback = elasticTable.getDoubleTopic("feedback").publish();
        m_kpfeedfoward = elasticTable.getDoubleTopic("feedforward").publish();
        feedforwardsubscriber = elasticTable.getDoubleTopic("feedforward").subscribe(0);
        feedbacksubscriber = elasticTable.getDoubleTopic("feedback").subscribe(0.5);
        subscriber_idkman_feedforwardsubscriber_publisher = elasticTable.getBooleanTopic("something_Normal").publish();
        subscriber_idkman_feedforwardsubscriber = elasticTable.getBooleanTopic("something_Normal").subscribe(false);
        m_filter_tx = elasticTable.getDoubleTopic("TxFilterRd").publish();
    }

    @Override
    public void periodic() {
        double filtered_tx = RobotContainer.getStableFilteredTx();
        m_taPub.set(LimelightHelpers.getTA("limelight"));
        m_txPub.set(LimelightHelpers.getTX("limelight"));
        m_tyPub.set(LimelightHelpers.getTY("limelight"));

        m_filter_tx.set(filtered_tx);

        feedback = feedbacksubscriber.get();
        feedforward = feedforwardsubscriber.get();
        Usefeedforward = subscriber_idkman_feedforwardsubscriber.get();
    }
}
