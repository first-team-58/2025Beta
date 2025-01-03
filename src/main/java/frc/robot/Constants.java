package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;

public class Constants {
    public static final boolean CALIBRATE = false;
    public static final boolean driverstationBool = DriverStation.getAlliance()
            .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;
    public static final double driverstationFlip = driverstationBool ? 1 : -1;

    public static class controllers {
        public static final CommandXboxController driver = new CommandXboxController(0);
    }

    public static class speeds {
        public static double MaxAcceleration = 10; // m/s TODO < no way im right
        public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // 5.21 m/s
        public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 rotation/sec
        public static double SlowSpeed = 3; // ms/s
        public static double deadband = 0.1 * MaxSpeed;
        public static double deadbandAngular = 0.1 * MaxAngularRate;
    }

    public static class field {
        public static final double fieldWidth = 16.57;

        /*
         * @param pose - pose to mirror across field midline
         */
        public static Pose2d mirrorPoseOnField(Pose2d pose) {
            double xDistanceToMid = (fieldWidth / 2) - pose.getX();
            double newX = (fieldWidth / 2) + xDistanceToMid;
            double newY = pose.getY();
            Rotation2d newAngle = Rotation2d.fromDegrees(180).minus(pose.getRotation());
            return new Pose2d(newX, newY, newAngle);
        }

        // speaker
        private static final Pose2d speaker_blue = new Pose2d(-0.0381, 5.547868, Rotation2d.fromDegrees(180));
        public static final Pose2d speaker = driverstationBool ? speaker_blue : mirrorPoseOnField(speaker_blue);

        // speaker shot
        private static final Pose2d speaker_shot_blue = new Pose2d(1.5, 5.5, Rotation2d.fromDegrees(0));
        public static final Pose2d speaker_shot = driverstationBool ? speaker_shot_blue
                : mirrorPoseOnField(speaker_shot_blue);

        // pass
        private static final Pose2d pass_blue = new Pose2d(1.5, 7.25, Rotation2d.fromDegrees(180));
        public static final Pose2d pass = driverstationBool ? pass_blue : mirrorPoseOnField(pass_blue);

        // pass shot
        private static final Pose2d pass_shot_blue = new Pose2d(9.78, 1.25, Rotation2d.fromDegrees(-40));
        public static final Pose2d pass_shot = driverstationBool ? pass_shot_blue : mirrorPoseOnField(pass_shot_blue);

        // amp shot
        private static final Pose2d amp_shot_blue = new Pose2d(2, 7.7, Rotation2d.fromDegrees(-90));
        public static final Pose2d amp_shot = driverstationBool ? amp_shot_blue : mirrorPoseOnField(amp_shot_blue);

        // zero
        private static final Pose2d zero_blue = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        public static final Pose2d zero = driverstationBool ? zero_blue : mirrorPoseOnField(zero_blue);
    }

    public static class oculus {
        public static final Transform2d robotToOculus = new Transform2d(
                // TODO
                // you can find this by setting both to zero, rotating 180, then dividing the
                // resulting pose by 2 for x and y
                new Translation2d(
                        Units.inchesToMeters(-9.45),
                        Units.inchesToMeters(3.937)),
                new Rotation2d(Units.degreesToRadians(0.0)));
        public static final Matrix<N3, N1> stddev = MatBuilder.fill(
                Nat.N3(),
                Nat.N1(),
                0.005, // x stddev in meters
                0.005, // y stddev in meters
                0.01 // heading stddev in radians
        );
    }

    public static class PID {
        public static class trans {

            // Example trapezoid profile constraints for translation
            public static final TrapezoidProfile.Constraints kConstraints = new TrapezoidProfile.Constraints(
                    speeds.MaxSpeed, // max velocity (m/s)
                    speeds.MaxAcceleration // max accel (m/s^2) - adjust as needed
            );

            // PID gains for translation
            public static final double Kp = 2.5;
            public static final double Ki = 0.0;
            public static final double Kd = 0.0;

            // Tolerances
            public static final double tolMeters = 0.03; // how close in XY to be "done"

        }

        public static class rot {
            public static final SwerveRequest.ForwardPerspectiveValue ForwardPerspective = SwerveRequest.ForwardPerspectiveValue.BlueAlliance;
            public static final double kP = 4;
            public static final double continous = Units.degreesToRadians(360);
            public static final double kTolRads = Units.degreesToRadians(1.0);

        }
    }
}
