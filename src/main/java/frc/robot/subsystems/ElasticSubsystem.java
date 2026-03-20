package frc.robot.subsystems;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// The ElasticSubsystem periodically publishes custom data to our NetworkTable for Elastic

public class ElasticSubsystem extends SubsystemBase{
    private final NetworkTable elasticTable;
    // field errors and distance errors are already put on smartdashboard
    private final DoublePublisher velocityPublisher;
    private final DoublePublisher voltagePublisher;
    private final Field2d m_field = new Field2d();
    private final Limelight limelight;
    // private final BooleanPublisher matchState_Publisher;
    // private final BooleanSubscriber matchState_Subscriber;



    public ElasticSubsystem(Limelight limelight) {
        this.limelight = limelight;
        // we need to fix matchstatemanager
        elasticTable = NetworkTableInstance.getDefault().getTable("elastic_datatable");
        velocityPublisher = elasticTable.getDoubleTopic("Velocity").publish();
        voltagePublisher = elasticTable.getDoubleTopic("Voltage").publish();

    }

    @Override
    public void periodic() {
        // periodically gets/sets data
        //boolean weWonAuto = ElasticSubsystem.getWeWonAuto();
        //weWonAuto_Publisher.set(weWonAuto);
        velocityPublisher.set(limelight.getBotSpeed());
        voltagePublisher.set(RobotController.getBatteryVoltage());

        SmartDashboard.putData("Field", m_field);
        m_field.setRobotPose(limelight.getBotPose());
    }
}
