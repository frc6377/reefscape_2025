package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import org.littletonrobotics.junction.Logger;

public class QuestNavSubsystem extends SubsystemBase {
  private final QuestNav questNav;
  private Pose3d robotPose;
  private Pose3d questPose;
  private static final Transform3d ROBOT_TO_QUEST =
      new Transform3d(0.0, 0.0, 0.0, new Rotation3d());
  private static final Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(0.02, 0.02, 0.035);

  public QuestNavSubsystem() {
    ;
    questNav = new QuestNav();
    robotPose = new Pose3d();
    questPose = robotPose.transformBy(ROBOT_TO_QUEST);
    questNav.setPose(questPose);
  }

  @Override
  public void periodic() {
    questNav.commandPeriodic(); // REQUIRED - Must be called every periodic cycle
    PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();
    for (PoseFrame questFrame : questFrames) {
      if (questFrame.isTracking()) {
        Pose3d questPose = questFrame.questPose3d();
        double timestamp = questFrame.dataTimestamp();
        Pose3d robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());
      }
    }
    Logger.recordOutput("Vision/QuestNav", questPose);
  }

  public void resetPose(Pose3d robotPose) {
    Pose3d questPose = robotPose.transformBy(ROBOT_TO_QUEST);
    questNav.setPose(questPose);
  }
}
