// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.RomiDrivetrainSubsystem;

public class CG_AutoDistance extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public CG_AutoDistance(RomiDrivetrainSubsystem drivetrain) {
    addCommands(
        new DriveDistance(0.5, 10, drivetrain),
        new TurnDegrees(0.3, 180, drivetrain),
        new DriveDistance(0.5, 10, drivetrain),
        new TurnDegrees(0.3, 180, drivetrain));
  }
}
