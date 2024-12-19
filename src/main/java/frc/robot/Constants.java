package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
        public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired
                                                                                            // top
        public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation/sec
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
    }
}
