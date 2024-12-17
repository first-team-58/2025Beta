package frc.robot.requests;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.generated.TunerConstants;

/**
 * A SwerveRequest that drives the robot to a given target pose using a simple
 * proportional feedback controller.
 * 
 * This request checks the current pose from SwerveControlParameters and
 * computes
 * a velocity command to move the robot toward the target. Once within
 * tolerance,
 * it applies an Idle request.
 */
public class DriveToPoseRequest implements SwerveRequest {

    private final Pose2d targetPose;

    // Simple gains and tolerances
    private static final double DRIVE_KP = 1.0; // Proportional gain for drive
    private static final double THETA_KP = 3.0; // Proportional gain for heading
    private static final double DRIVE_TOLERANCE = 0.05; // meters
    private static final double THETA_TOLERANCE = Math.toRadians(2.0); // radians

    /**
     * Constructs a request to drive to the specified target pose.
     * 
     * @param targetPose The desired final pose of the robot.
     */
    public DriveToPoseRequest(Pose2d targetPose) {
        this.targetPose = targetPose;
    }

    @Override
    public StatusCode apply(SwerveControlParameters parameters,
            com.ctre.phoenix6.swerve.SwerveModule... modulesToApply) {
        Pose2d currentPose = parameters.currentPose;

        // Calculate translation error
        double dx = targetPose.getX() - currentPose.getX();
        double dy = targetPose.getY() - currentPose.getY();
        double distanceError = Math.sqrt(dx * dx + dy * dy);

        // Calculate heading error
        double currentHeading = currentPose.getRotation().getRadians();
        double targetHeading = targetPose.getRotation().getRadians();
        double headingError = targetHeading - currentHeading;
        // Normalize heading error to range (-pi, pi)
        headingError = Math.atan2(Math.sin(headingError), Math.cos(headingError));

        // Check if we are close enough
        if (distanceError < DRIVE_TOLERANCE && Math.abs(headingError) < THETA_TOLERANCE) {
            return new Idle().apply(parameters, modulesToApply);
        }

        // Compute speed commands
        double XSpeed = clamp(DRIVE_KP * dx, -1.0, 1.0) * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        double YSpeed = clamp(DRIVE_KP * dy, -1.0, 1.0) * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        double RotSpeed = clamp(THETA_KP * headingError, -2.0, 2.0);

        // Apply a field-centric request with the computed velocities
        return new FieldCentric()
                .withVelocityX(XSpeed)
                .withVelocityY(YSpeed)
                .withRotationalRate(RotSpeed)
                .apply(parameters, modulesToApply);
    }

    /**
     * Utility method to clamp a value between min and max.
     */
    private static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(val, max));
    }
}
