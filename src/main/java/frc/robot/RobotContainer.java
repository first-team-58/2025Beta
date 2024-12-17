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
import frc.robot.Constants.controllers;
import frc.robot.Constants.field;
import frc.robot.generated.TunerConstants;
import frc.robot.requests.DriveToPoseRequest;
import frc.robot.requests.DriveWithTargetRequest;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 rev/sec
  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive

  private final DriveToPoseRequest driveToTarget = new DriveToPoseRequest();

  private final DriveWithTargetRequest driveWithTargetRequest = new DriveWithTargetRequest();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  /* Path follower */
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser("Tests");
    SmartDashboard.putData("Auto Mode", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    // normal
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> drive
            .withVelocityX(-controllers.driver.getLeftY() * MaxSpeed)
            .withVelocityY(-controllers.driver.getLeftX() * MaxSpeed)
            .withRotationalRate(-controllers.driver.getRightX() * MaxAngularRate)));

    // amp
    controllers.driver.leftTrigger().whileTrue(
        drivetrain.applyRequest(() -> driveToTarget.withPose(field.amp_shot)));

    // speaker
    controllers.driver.rightTrigger().whileTrue(drivetrain.applyRequest(() -> driveToTarget
        .withPose(field.speaker_shot)));

    // pass
    controllers.driver.leftBumper().whileTrue(drivetrain.applyRequest(() -> driveToTarget.withPose(field.pass_shot)));

    // collect
    controllers.driver.rightBumper().whileTrue(
        drivetrain.applyRequest(() -> driveWithTargetRequest
            .withVelocityX(-controllers.driver.getLeftY() * MaxSpeed)
            .withVelocityY(-controllers.driver.getLeftX() * MaxSpeed)
            .withRotationalRate(-controllers.driver.getRightX() * MaxAngularRate)
            .withTx(10)));

    // reset pose
    controllers.driver.back()
        .onTrue(drivetrain.runOnce(() -> drivetrain.resetPose(new Pose2d(0, 0, new Rotation2d(0)))));

    // reset heading
    controllers.driver.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return autoChooser.getSelected();
  }
}
