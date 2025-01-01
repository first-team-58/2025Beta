package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class riot_util {

    public static Rotation2d angleToPose(Pose2d robotPose, Translation2d targetPose) {
        Translation2d currentPosition = robotPose.getTranslation();

        // Compute the angle
        Rotation2d angle = currentPosition.minus(
                targetPose)
                .getAngle()
                .plus(Rotation2d.fromDegrees(180));

        return angle;
    }

    /**
     * Returns a heading error in (-pi, pi).
     */
    public static double getHeadingError(double currentHeading, double targetHeading) {
        double error = targetHeading - currentHeading;
        return Math.atan2(Math.sin(error), Math.cos(error));
    }

    /**
     * Utility method to clamp a value between min and max.
     */
    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(val, max));
    }
}
