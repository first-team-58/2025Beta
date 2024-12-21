// stolen from outliers, thank you freyja
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
import frc.robot.Constants.oculus;

public class OculusProcessor {

  // Configure Network Tables topics (oculus/...) to communicate with the Quest
  // HMD
  NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault();
  NetworkTable nt4Table = nt4Instance.getTable("oculus");
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

  public OculusProcessor() {

  }

  public float yaw_offset = 0.0f;

  public float getOculusYaw() {
    float[] eulerAngles = questEulerAngles.get();
    return eulerAngles[1] - yaw_offset;
  }

  public Translation2d getOculusPosition() {
    float[] oculusPosition = questPosition.get();
    return new Translation2d(-oculusPosition[2], oculusPosition[0]);
  }

  public Pose2d getOculusPose() {
    return new Pose2d(getOculusPosition(), Rotation2d.fromDegrees(-getOculusYaw()));
  }

  public Pose2d getRobotPose() {
    var oculusToRobot = oculus.robotToOculus.inverse();
    Pose2d robotPose = getOculusPose().transformBy(oculusToRobot);
    return robotPose;
  }

  public double getTimestamp() {
    return questTimestamp.get();
  }

  // TODO: do we need a red flip?
  // public Pose2d getOculusPose(){
  // // if (_driveTrain.isRedAlliance()) {
  // // Pose2d initialPose =
  // (firstPath.flipPath().getPreviewStartingHolonomicPose());
  // // } else {
  // // Pose2d initialPose = (firstPath.getPreviewStartingHolonomicPose());
  // // }
  // return
  // }
}
