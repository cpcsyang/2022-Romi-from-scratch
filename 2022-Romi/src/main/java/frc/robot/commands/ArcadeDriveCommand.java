// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.RomiDrivetrainSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

  /**
   * Creates a new ArcadeDrive. This command will drive your robot according to the speed supplier
   * lambdas. This command does not terminate.
   *
   * @param m_drivetrain The drivetrain subsystem on which this command will run
   * @param m_forwardSpeedSupplier Lambda supplier of forward/backward speed
   * @param m_RotateSpeedSuppplier Lambda supplier of rotational speed
   */
public class ArcadeDriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final RomiDrivetrainSubsystem m_drivetrain;
  private final Supplier<Double> m_forwardSpeedSupplier;
  private final Supplier<Double> m_rotateSpeedSupplier;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArcadeDriveCommand(
        RomiDrivetrainSubsystem drivetrainSubsystem,
        Supplier<Double> forwardSpeedSupplier,
        Supplier<Double> rotateSpeedSupplier) {
    m_drivetrain = drivetrainSubsystem;
    m_forwardSpeedSupplier = forwardSpeedSupplier;
    m_rotateSpeedSupplier = rotateSpeedSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("----------> ArcadeDriveCommand initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.arcadeDrive(m_forwardSpeedSupplier.get(), m_rotateSpeedSupplier.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("----------> ArcadeDriveCommand end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
