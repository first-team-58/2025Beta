package frc.robot.requests;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.util.riot_angles;

public class DriveToPoseRequest implements SwerveRequest {

    private Pose2d targetPose;

    // Pull in speed/accel limits from your existing constants:
    private static final double kMaxTransSpeed = TunerConstants.kSpeedAt12Volts
            .in(edu.wpi.first.units.Units.MetersPerSecond);

    // Example trapezoid profile constraints for translation
    private static final TrapezoidProfile.Constraints kTransConstraints = new TrapezoidProfile.Constraints(
            kMaxTransSpeed, // max velocity (m/s)
            kMaxTransSpeed // max accel (m/s^2) - adjust as needed
    );

    // PID gains for translation
    private static final double kTransKp = 1.0;
    private static final double kTransKi = 0.0;
    private static final double kTransKd = 0.0;

    // Tolerances
    private static final double kDriveToleranceMeters = 0.05; // how close in XY to be "done"
    private static final double kRotToleranceRadians = Units.degreesToRadians(2.0);

    // Create separate controllers for X, Y, and Heading
    private final ProfiledPIDController xController = new ProfiledPIDController(
            kTransKp, kTransKi, kTransKd, kTransConstraints);

    private final ProfiledPIDController yController = new ProfiledPIDController(
            kTransKp, kTransKi, kTransKd, kTransConstraints);

    public DriveToPoseRequest() {
    }

    public DriveToPoseRequest withPose(Pose2d pose) {
        this.targetPose = pose;
        return this;
    }

    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule... modulesToApply) {
        Pose2d currentPose = parameters.currentPose;

        // 1) Check distance & heading error
        double distanceError = currentPose.getTranslation()
                .getDistance(targetPose.getTranslation());
        double headingError = riot_angles.getHeadingError(
                currentPose.getRotation().getRadians(),
                targetPose.getRotation().getRadians());

        if (distanceError < kDriveToleranceMeters && Math.abs(headingError) < kRotToleranceRadians) {
            // We are close enough: apply Idle request
            return new Idle().apply(parameters, modulesToApply);
        }

        // 2) Update setpoints
        xController.setGoal(targetPose.getX());
        yController.setGoal(targetPose.getY());

        // 3) Calculate the output velocities
        double xSpeed = xController.calculate(currentPose.getX());
        double ySpeed = yController.calculate(currentPose.getY());

        // 4) Clamp speeds if desired (avoid extreme overshoot)
        xSpeed = riot_angles.clamp(xSpeed, -kMaxTransSpeed, kMaxTransSpeed);
        ySpeed = riot_angles.clamp(ySpeed, -kMaxTransSpeed, kMaxTransSpeed);

        // 5) Build the request using FieldCentricFacingAngle
        return new SwerveRequest.FieldCentricFacingAngle()
                // Flip if needed for driver station perspective
                .withVelocityX(xSpeed * Constants.driverstationFlip)
                .withVelocityY(ySpeed * Constants.driverstationFlip)
                // We want to face the target heading
                .withTargetDirection(Rotation2d.fromRadians(targetPose.getRotation().getRadians()))
                .apply(parameters, modulesToApply);
    }
}
