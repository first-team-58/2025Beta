package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.field;

public class DriveHelper {
    public static Rotation2d angleToSpeaker(Pose2d robotPose, ChassisSpeeds speeds) {
        Translation2d speakerPos = field.speaker.getTranslation();
        Translation2d currentPosition = robotPose.getTranslation();

        // Compute the angle from the speaker to the robot
        Rotation2d angle = currentPosition.minus(speakerPos)
                .getAngle()
                .plus(Rotation2d.fromDegrees(180));

        return angle;
    }

    public static Rotation2d angleToPass(Pose2d robotPose, ChassisSpeeds speeds) {
        Translation2d passPos = field.pass.getTranslation();
        Translation2d currentPosition = robotPose.getTranslation();

        // Compute the angle from the speaker to the predicted future position.
        Rotation2d angle = currentPosition.minus(
                passPos)
                .getAngle()
                .plus(Rotation2d.fromDegrees(180));

        return angle;
    }

}
