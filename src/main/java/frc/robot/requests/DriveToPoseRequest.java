package frc.robot.requests;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.Constants.PID;
import frc.robot.Constants.speeds;
import frc.robot.util.riot_angles;

public class DriveToPoseRequest implements SwerveRequest {

        private Pose2d targetPose;
        private final SwerveRequest.FieldCentricFacingAngle facingAngle = new SwerveRequest.FieldCentricFacingAngle();

        // Create separate controllers for X and Y
        private final ProfiledPIDController xController = new ProfiledPIDController(
                        PID.trans.Kp, PID.trans.Ki, PID.trans.Kd, PID.trans.kConstraints);

        private final ProfiledPIDController yController = new ProfiledPIDController(
                        PID.trans.Kp, PID.trans.Ki, PID.trans.Kd, PID.trans.kConstraints);

        public DriveToPoseRequest() {
                facingAngle.ForwardPerspective = PID.rot.ForwardPerspective;
                facingAngle.HeadingController.setP(PID.rot.kP);
                facingAngle.HeadingController.enableContinuousInput(0, PID.rot.continous);
                facingAngle.HeadingController.setTolerance(PID.rot.kTolRads);
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

                if (distanceError < PID.trans.tolMeters && Math.abs(headingError) < PID.rot.kTolRads) {
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
                xSpeed = riot_angles.clamp(xSpeed, -speeds.MaxSpeed, speeds.MaxSpeed);
                ySpeed = riot_angles.clamp(ySpeed, -speeds.MaxSpeed, speeds.MaxSpeed);

                // 5) Build the request using FieldCentricFacingAngle
                return facingAngle
                                // Flip if needed for driver station perspective
                                .withVelocityX(xSpeed * Constants.driverstationFlip)
                                .withVelocityY(ySpeed * Constants.driverstationFlip)
                                // We want to face the target heading
                                .withTargetDirection(Rotation2d.fromRadians(targetPose.getRotation().getRadians()))
                                .apply(parameters, modulesToApply);
        }
}
