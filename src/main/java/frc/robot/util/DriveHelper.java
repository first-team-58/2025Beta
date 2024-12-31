package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class DriveHelper {
    public static Rotation2d angleToPose(Pose2d robotPose, Translation2d targetPose) {
        Translation2d currentPosition = robotPose.getTranslation();

        // Compute the angle
        Rotation2d angle = currentPosition.minus(
                targetPose)
                .getAngle()
                .plus(Rotation2d.fromDegrees(180));

        return angle;
    }

}
