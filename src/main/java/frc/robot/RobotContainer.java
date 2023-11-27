// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SwerveDefault;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  XboxController driverController = new XboxController(0);

  private final SwerveSubsystem driveSubsystem = new SwerveSubsystem();

  public RobotContainer() {
    configureBindings();

    SwerveDefault defaultTeleopDrive = new SwerveDefault(
        driveSubsystem,
        () -> MathUtil.applyDeadband(-driverController.getLeftY(),
            0.1),
        () -> MathUtil.applyDeadband(-driverController.getLeftX(),
            0.1),
        () -> MathUtil.applyDeadband(-driverController.getRightX(), 0.1),
        () -> true);

    driveSubsystem.setDefaultCommand(defaultTeleopDrive);
  }

  private void configureBindings() {
    new JoystickButton(driverController, 1).onTrue((new InstantCommand(driveSubsystem::zeroGyro)));
    new JoystickButton(driverController, 3).onTrue(new InstantCommand(driveSubsystem::addFakeVisionReading));
  }

  public Command getAutonomousCommand() {
    return new PrintCommand("No Auto");
  }
}
