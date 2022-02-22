// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.subsystems.RomiDrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // @SuppressWarnings("unused")
  private final XboxController m_controller;  // TODO: just to see if this breaks the script
  private final RomiDrivetrainSubsystem m_romiDrivetrain;;
  // @SuppressWarnings("unused")
  private final AutoDriveCommand m_autoCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // initialize controller objects
    m_controller = new XboxController(0);

    // initialize subsystems objects
    m_romiDrivetrain = new RomiDrivetrainSubsystem();

    // initialize commands objects
    m_autoCommand = new AutoDriveCommand(m_romiDrivetrain);

    // set default command for all subsystems
    m_romiDrivetrain.setDefaultCommand(getArcadeDriveCommand());

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }

  public Command getArcadeDriveCommand() {
    return new ArcadeDriveCommand(    // 1 is Left-Y-Axis; negate stick-Y-axis   // 4 is Right-X-Axis
      m_romiDrivetrain, () -> -m_controller.getRawAxis(1),  () -> m_controller.getRawAxis(4));
  }
}
