package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.oculus;

public class OculusProcessor {

  // Configure Network Tables topics (oculus/...) to communicate with the Quest
  NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault();
  NetworkTable nt4Table = nt4Instance.getTable("questnav");

  public IntegerSubscriber questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);
  public IntegerPublisher questMosi = nt4Table.getIntegerTopic("mosi").publish();

  // Subscribe to the Network Tables oculus data topics
  public IntegerSubscriber questFrameCount = nt4Table.getIntegerTopic("frameCount").subscribe(0);
  public DoubleSubscriber questTimestamp = nt4Table.getDoubleTopic("timestamp").subscribe(0.0f);
  public FloatArraySubscriber questPosition = nt4Table.getFloatArrayTopic("position")
      .subscribe(new float[] { 0.0f, 0.0f, 0.0f });
  public FloatArraySubscriber questQuaternion = nt4Table.getFloatArrayTopic("quaternion")
      .subscribe(new float[] { 0.0f, 0.0f, 0.0f, 0.0f });
  public FloatArraySubscriber questEulerAngles = nt4Table.getFloatArrayTopic("eulerAngles")
      .subscribe(new float[] { 0.0f, 0.0f, 0.0f });
  public DoubleSubscriber questBattery = nt4Table.getDoubleTopic("batteryPercent").subscribe(0.0f);

  private final StructPublisher<Pose2d> questnavRobotPose = nt4Table.getStructTopic("robotPose", Pose2d.struct)
      .publish();
  private final StringPublisher questnavTypePub = nt4Table.getStringTopic(".type").publish();

  public OculusProcessor() {
    configureShuffleboard();
  }

  public float yaw_offset = 0.0f;

  public float getOculusYaw() {
    float[] eulerAngles = questEulerAngles.get();
    return eulerAngles[1] - yaw_offset;
  }

  public Translation2d getOculusPosition() {
    float[] oculusPosition = questPosition.get();
    return new Translation2d(
        oculusPosition[0],
        oculusPosition[2]);
  }

  public Pose2d getOculusPose() {
    return new Pose2d(getOculusPosition(), Rotation2d.fromDegrees(-getOculusYaw()));
  }

  public Pose2d getRobotPose() {
    var oculusToRobot = oculus.robotToOculus.inverse();
    return getOculusPose().transformBy(oculusToRobot);
  }

  public double getTimestamp() {
    return questTimestamp.get();
  }

  public void configureShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("Oculus");
    tab.addNumber("frameCount", () -> questFrameCount.get());
    tab.addNumber("timestamp", () -> questTimestamp.get());
    tab.addNumber("battery", () -> questBattery.get());

    questnavTypePub.set("Field2d");
    questnavRobotPose.set(getRobotPose());
  }

  public void updateQuestnavPose() {
    questnavRobotPose.set(getRobotPose());
  }

  public void zeroPosition() {
    if (questMiso.get() != 99) {
      questMosi.set(1);
    }
  }

  public void resetMosi() {
    if (questMiso.get() == 99) {
      questMosi.set(0);
    }
  }
}
