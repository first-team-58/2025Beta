package frc.robot.requests;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * This request restricts translation to only forward/backward relative to the
 * robot's heading and adds a lateral correction based on a target offset (tx).
 * 
 * Usage:
 * 
 * <pre>
 *   DriveWithTargetRequest driveWithTarget = new DriveWithTargetRequest()
 *       .withVelocityX(... )   // field-centric X velocity (m/s)
 *       .withVelocityY(... )   // field-centric Y velocity (m/s)
 *       .withRotationalRate(...) // rotational rate (rad/s)
 *       .withTx(... ); // tx from vision system
 * </pre>
 */
public class DriveWithTargetRequest implements SwerveRequest {

    private double velocityX = 0.0;
    private double velocityY = 0.0;
    private double rotationalRate = 0.0;
    private double tx = 0.0;

    // This is the maximum speed and rotation speed defined elsewhere
    // Adjust if needed or leave as is.
    private static final double TX_KP = 0.15; // Lateral gain from tx

    /**
     * Sets the supplier for field-centric X velocity (m/s).
     */
    public DriveWithTargetRequest withVelocityX(double velocityX) {
        this.velocityX = velocityX;
        return this;
    }

    /**
     * Sets the supplier for field-centric Y velocity (m/s).
     */
    public DriveWithTargetRequest withVelocityY(double velocityY) {
        this.velocityY = velocityY;
        return this;
    }

    /**
     * Sets the for rotational rate (rad/s).
     */
    public DriveWithTargetRequest withRotationalRate(double rotationalRate) {
        this.rotationalRate = rotationalRate;
        return this;
    }

    /**
     * Sets the for tx offset (degrees from vision system).
     */
    public DriveWithTargetRequest withTx(double tx) {
        this.tx = tx;
        return this;
    }

    @Override
    public StatusCode apply(SwerveControlParameters parameters,
            com.ctre.phoenix6.swerve.SwerveModule... modulesToApply) {
        Pose2d currentPose = parameters.currentPose;

        // Robot heading
        double theta = currentPose.getRotation().getRadians();

        // Robot's forward axis in field coordinates
        double fwdX = Math.cos(theta);
        double fwdY = Math.sin(theta);

        // Project the commanded velocity onto the robot's forward axis
        double projection = velocityX * fwdX + velocityY * fwdY;
        double finalXSpeed = projection * fwdX;
        double finalYSpeed = projection * fwdY;

        // Compute lateral correction from tx
        double lateralCorrection = TX_KP * tx;

        // Lateral direction is perpendicular to forward: if forward is (fwdX, fwdY),
        // then lateral is (-fwdY, fwdX).
        double latX = fwdY;
        double latY = -fwdX;

        // Add lateral correction
        finalXSpeed += lateralCorrection * latX;
        finalYSpeed += lateralCorrection * latY;

        // Rotation from the driver
        double finalRotSpeed = rotationalRate;

        // Apply using a FieldCentric request internally
        return new FieldCentric()
                .withVelocityX(finalXSpeed)
                .withVelocityY(finalYSpeed)
                .withRotationalRate(finalRotSpeed)
                .apply(parameters, modulesToApply);
    }
}
