// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.requests.DriveToPoseRequest;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.DriveHelper;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                // speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                    // second
                                                                                    // max angular velocity
  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                                                                                 // // TODO:
                                                                                 // maybe less
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                               // motors

  private final SwerveRequest.FieldCentricFacingAngle facingAngle = new SwerveRequest.FieldCentricFacingAngle();
  // Create and apply the request
  Pose2d target = new Pose2d(3.0, 2.0, new Rotation2d(Math.toRadians(90)));
  DriveToPoseRequest driveToPoseRequest = new DriveToPoseRequest(target);

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController driverController = new CommandXboxController(0);
  // private final CommandXboxController driverController = new
  // CommandXboxController(0);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  /* Path follower */
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    // Set PID gains
    facingAngle.ForwardPerspective = SwerveRequest.ForwardPerspectiveValue.BlueAlliance;
    facingAngle.HeadingController.setP(4);
    facingAngle.HeadingController.enableContinuousInput(0, Units.degreesToRadians(360));
    facingAngle.HeadingController.setTolerance(Units.degreesToRadians(2.0));

    autoChooser = AutoBuilder.buildAutoChooser("Tests");
    SmartDashboard.putData("Auto Mode", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    // drivetrain.setDefaultCommand(
    // // Drivetrain will execute this command periodically
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> drive
            .withVelocityX(-driverController.getLeftY() * MaxSpeed)
            .withVelocityY(-driverController.getLeftX() * MaxSpeed)
            .withRotationalRate(-driverController.getRightX() * MaxAngularRate)));

    // Face amp
    driverController.b().whileTrue(
        drivetrain.applyRequest(
            () -> facingAngle
                .withVelocityX(-driverController.getLeftY() * MaxSpeed)
                .withVelocityY(-driverController.getLeftX() * MaxSpeed)
                .withTargetDirection(Rotation2d.fromDegrees(
                    Constants.driverstationFlip *
                        90))));

    // aim at speaker
    driverController.leftTrigger().whileTrue(
        drivetrain.applyRequest(
            () -> facingAngle
                .withVelocityX(-driverController.getLeftY() * MaxSpeed)
                .withVelocityY(-driverController.getLeftX() * MaxSpeed)
                .withTargetDirection(
                    DriveHelper.angleToSpeaker(drivetrain
                        .getState().Pose,
                        drivetrain.getState().Speeds))));

    // **New Code Here**: Pressing POV left triggers a command to drive to (2,2)
    driverController.povLeft().whileTrue(drivetrain.applyRequest(() -> driveToPoseRequest));

    driverController.back().onTrue(drivetrain.runOnce(() -> drivetrain.resetPose(new Pose2d(0, 0, new Rotation2d(0)))));

    // reset the field-centric heading on start button
    driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return autoChooser.getSelected();
  }
}
