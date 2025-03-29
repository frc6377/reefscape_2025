package frc.robot.subsystems.QuestNav;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class QuestNav {
  // Configure Network Tables topics (questnav/...) to communicate with the Quest HMD
  NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault();
  NetworkTable nt4Table = nt4Instance.getTable("questnav");
  private IntegerSubscriber questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);
  private IntegerPublisher questMosi = nt4Table.getIntegerTopic("mosi").publish();

  // Subscribe to the Network Tables questnav data topics
  private DoubleSubscriber questTimestamp = nt4Table.getDoubleTopic("timestamp").subscribe(0.0f);
  private FloatArraySubscriber questPosition =
      nt4Table.getFloatArrayTopic("position").subscribe(new float[] {0.0f, 0.0f, 0.0f});
  private FloatArraySubscriber questQuaternion =
      nt4Table.getFloatArrayTopic("quaternion").subscribe(new float[] {0.0f, 0.0f, 0.0f, 0.0f});
  private FloatArraySubscriber questEulerAngles =
      nt4Table.getFloatArrayTopic("eulerAngles").subscribe(new float[] {0.0f, 0.0f, 0.0f});
  private IntegerSubscriber questFrameCount = nt4Table.getIntegerTopic("frameCount").subscribe(0);
  private DoubleSubscriber questBatteryPercent =
      nt4Table.getDoubleTopic("device/batteryPercent").subscribe(0.0f);
  private BooleanSubscriber questIsTracking =
      nt4Table.getBooleanTopic("device/isTracking").subscribe(false);
  private IntegerSubscriber questTrackingLostCount =
      nt4Table.getIntegerTopic("device/trackingLostCounter").subscribe(0);

  /** Subscriber for heartbeat requests */
  private final DoubleSubscriber heartbeatRequestSub =
      nt4Table.getDoubleTopic("heartbeat/quest_to_robot").subscribe(0.0);

  /** Publisher for heartbeat responses */
  private final DoublePublisher heartbeatResponsePub =
      nt4Table.getDoubleTopic("heartbeat/robot_to_quest").publish();

  /** Last processed heartbeat request ID */
  private double lastProcessedHeartbeatId = 0;

  // Local heading helper variables
  private float yaw_offset = 0.0f;
  private Pose2d resetPosition = new Pose2d();
  private Pose2d startingPosition = new Pose2d();

  /** Process heartbeat requests from Quest and respond with the same ID */
  public void processHeartbeat() {
    double requestId = heartbeatRequestSub.get();
    // Only respond to new requests to avoid flooding
    if (requestId > 0 && requestId != lastProcessedHeartbeatId) {
      heartbeatResponsePub.set(requestId);
      lastProcessedHeartbeatId = requestId;
    }
  }

  // Gets the Quest's measured position.
  public Pose2d getPose() {
    return new Pose2d(
      getQuestNavPose()
        .minus(resetPosition)
        .getTranslation()
        .plus(startingPosition.getTranslation()),
      Rotation2d.fromDegrees(getOculusYaw()));
  }
  public Pose2d getRawPose() {
    return getQuestNavPose();
  }
  public Pose2d getOdometryPose() {
    return new Pose2d(getQuestNavPose().minus(resetPosition).getTranslation(), Rotation2d.fromDegrees(getOculusYaw()));
  }
  // Gets the battery percent of the Quest.
  public Double getBatteryPercent() {
    return questBatteryPercent.get();
  }
  public void setPose(Pose2d pose) {
    startingPosition = pose;
  }
  // Gets the current tracking state of the Quest.
  public Boolean getTrackingStatus() {
    return questIsTracking.get();
  }

  // Gets the current frame count from the Quest headset.
  public Long getFrameCount() {
    return questFrameCount.get();
  }

  // Gets the number of tracking lost events since the Quest connected to the robot.
  public Long getTrackingLostCounter() {
    return questTrackingLostCount.get();
  }

  // Returns if the Quest is connected.
  public Boolean connected() {
    return ((RobotController.getFPGATime() - questBatteryPercent.getLastChange()) / 1000) < 250;
  }

  // Gets the Quaternion of the Quest.
  public Quaternion getQuaternion() {
    float[] qqFloats = questQuaternion.get();
    return new Quaternion(qqFloats[0], qqFloats[1], qqFloats[2], qqFloats[3]);
  }

  // Gets the Quests's timestamp in NT Server Time.
  public double timestamp() {
    return questTimestamp.getAtomic().serverTime;
  }

  // Zero the relativerobot heading
  public void zeroHeading() {
    float[] eulerAngles = questEulerAngles.get();
    yaw_offset = eulerAngles[1];
  }

  // Zero the absolute 3D position of the robot (similar to long-pressing the quest logo)
  public void zeroPosition() {
    resetPosition = getPose();
    if (questMiso.get() != 99) {
      questMosi.set(1);
    }
  }

  // Clean up questnav subroutine messages after processing on the headset
  public void cleanUpQuestNavMessages() {
    if (questMiso.get() == 99) {
      questMosi.set(0);
    }
  }

  // Get the yaw Euler angle of the headset
  private float getOculusYaw() {
    float[] eulerAngles = questEulerAngles.get();
    var ret = eulerAngles[1] - yaw_offset;
    ret %= 360;
    if (ret < 0) {
      ret += 360;
    }
    return ret;
  }

  private Translation2d getQuestNavTranslation() {
    float[] questnavPosition = questPosition.get();
    return new Translation2d(questnavPosition[2], -questnavPosition[0]);
  }

  private Pose2d getQuestNavPose() {
    var oculousPositionCompensated =
        getQuestNavTranslation().minus(new Translation2d(0, 0.1651)); // 6.5
    return new Pose2d(oculousPositionCompensated, Rotation2d.fromDegrees(getOculusYaw()));
  }
}
