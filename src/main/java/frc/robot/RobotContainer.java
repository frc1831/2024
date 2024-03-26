// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.CompetitionVariables.*;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SpinCommand;
import frc.robot.drivebase.TeleopDrive;
import frc.robot.subsystems.SwerveBase;
import frc.robot.subsystems.SpinMotor;

public class RobotContainer {
  private final SwerveBase drivebase = new SwerveBase();
  CommandXboxController driverController = new CommandXboxController(0);
  CommandXboxController operatorController = new CommandXboxController(1);
  private final SlewRateLimiter forAccel = new SlewRateLimiter(8);
  private final SlewRateLimiter starAccel = new SlewRateLimiter(8);
  private final SlewRateLimiter rotAccel = new SlewRateLimiter(30);

  public RobotContainer() {
    CameraServer.startAutomaticCapture();

    configureBindings();

    TeleopDrive openRobotRel = new TeleopDrive(
        drivebase,
        () -> forAccel.calculate((Math.abs(driverController.getLeftY()) > 0.1) ? driverController.getLeftY() : 0),
        () -> starAccel.calculate((Math.abs(driverController.getLeftX()) > 0.1) ? driverController.getLeftX() : 0),
        () -> rotAccel.calculate(driverController.getRightX()),
        () -> false, true);

    TeleopDrive closedRobotRel = new TeleopDrive(
        drivebase,
        () -> (Math.abs(driverController.getLeftY()) > 0.1) ? -driverController.getLeftY() : 0,
        () -> (Math.abs(driverController.getLeftX()) > 0.1) ? -driverController.getLeftX() : 0,
        () -> -driverController.getRightX(),
        () -> false, false);

    TeleopDrive openFieldRel = new TeleopDrive(
        drivebase,
        () -> (Math.abs(driverController.getLeftY()) > 0.10) ? -driverController.getLeftY() : 0,
        () -> (Math.abs(driverController.getLeftX()) > 0.10) ? -driverController.getLeftX() : 0,
        () -> driverController.getRightX(), () -> true, true);

    TeleopDrive closedFieldRel = new TeleopDrive(
        drivebase,
        () -> (Math.abs(driverController.getLeftY()) > 0.05) ? -driverController.getLeftY() : 0,
        () -> (Math.abs(driverController.getLeftX()) > 0.05) ? -driverController.getLeftX() : 0,
        () -> driverController.getRightX(), () -> true, false);

    drivebase.setDefaultCommand(closedRobotRel);
  }

  private final SpinMotor collectorTop = new SpinMotor(53);
  private final SpinMotor collectorBottom = new SpinMotor(51);
  private final SpinCommand collectorCommand = new SpinCommand.Builder()
      .motors(COLLECTOR_SPEED, collectorTop, collectorBottom)
      .build();

  private final SpinMotor climberLeft = new SpinMotor(52);
  private final SpinMotor climberRight = new SpinMotor(54);
  private final SpinCommand climberCommand = new SpinCommand.Builder()
      .motor(climberLeft, -CLIMBER_SPEED)
      .motor(climberRight, CLIMBER_SPEED)
      .build();

  private final SpinCommand climberLeftCommand = new SpinCommand.Builder()
      .motor(climberLeft, CLIMBER_SPEED)
      .build();
  private final SpinCommand climberRightCommand = new SpinCommand.Builder()
      .motor(climberRight, -CLIMBER_SPEED).build();

  private final SpinMotor feederRoller = new SpinMotor(55);
  private final SpinMotor rightShooter = new SpinMotor(56);
  private final SpinMotor leftShooter = new SpinMotor(57);

  private final SpinCommand highShooterCommand = new SpinCommand.Builder()
      .motor(rightShooter, FAST_SHOOTER_SPEED)
      .motor(leftShooter, -FAST_SHOOTER_SPEED)
      .delayChain().link(FAST_SHOOTER_DWELL_TIME, collectorTop, feederRoller)
      .withSpeeds(FEEDER_COLLECTOR_SPEED, FEEDER_ROLLER_SPEED).endChain()
      .build();
  private final SpinCommand lowShooterCommand = new SpinCommand.Builder()
      .motor(rightShooter, SLOW_SHOOTER_SPEED)
      .motor(leftShooter, -SLOW_SHOOTER_SPEED)
      .delayChain().link(SLOW_SHOOTER_DWELL_TIME, collectorTop, feederRoller)
      .withSpeeds(FEEDER_COLLECTOR_SPEED, FEEDER_ROLLER_SPEED).endChain()
      .build();

  private void configureBindings() {
    // ======================================================================================================================
    // MISCELLANEOUS COMMANDS
    // ======================================================================================================================
    driverController.rightBumper().whileTrue(collectorCommand);
    driverController.leftBumper().whileTrue(highShooterCommand);
    // driveController.leftTrigger().whileTrue(lowShooterCommand);

    operatorController.rightBumper().whileTrue(climberCommand);
    operatorController.leftBumper().whileTrue(climberCommand.toInverted());

    operatorController.rightTrigger().whileTrue(climberRightCommand);
    operatorController.leftTrigger().whileTrue(climberLeftCommand);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
