package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Milliamp;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

// The ElasticSubsystem periodically publishes custom data to our NetworkTable for Elastic

public class ElasticSubsystem extends SubsystemBase{
    private final NetworkTable elasticTable;
    private final BooleanPublisher weWonAuto_Publisher;
    private final BooleanPublisher teamHubActive_Publisher;
    private final IntegerPublisher fieldError_Publisher;
    private final IntegerPublisher distanceError_Publisher;
    // private final BooleanPublisher matchState_Publisher;
    // private final BooleanSubscriber matchState_Subscriber;



    public ElasticSubsystem() {
        elasticTable = NetworkTableInstance.getDefault().getTable("elastic_datatable");
        weWonAuto_Publisher = elasticTable.getBooleanTopic("weWonAuto").publish();
        teamHubActive_Publisher = elasticTable.getBooleanTopic("teamHubActive").publish();
        fieldError_Publisher = elasticTable.getIntegerTopic("fieldError").publish();
        distanceError_Publisher = elasticTable.getIntegerTopic("distanceError").publish();
    }

    // ahadu, who won auto has been moved to robot.java for simplicity so we dont call it 1,000,000 times
    // it gets set once teleopinit starts, refrence it from Constants.WE_WON_AUTO

    @Override
    public void periodic() {
        // periodically gets/sets data
        boolean matchstate = MatchStateManager.getActive();
        //boolean weWonAuto = ElasticSubsystem.getWeWonAuto();
        //weWonAuto_Publisher.set(weWonAuto);
        teamHubActive_Publisher.set(matchstate);
        fieldError_Publisher.set(Limelight.fieldError);
        distanceError_Publisher.set(Limelight.distanceError);
    }
}
