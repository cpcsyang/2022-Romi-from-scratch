// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.romi.OnBoardIO;
import edu.wpi.first.wpilibj.romi.OnBoardIO.ChannelMode;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.CG_AutoTime;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.TurnDegrees;
import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.commands.CG_AutoDistance;
import frc.robot.subsystems.RomiDrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // drivetrain constants
  public static final double ksVolts = 0.929;
  public static final double kvVoltSecondsPerMeter = 6.33;
  public static final double kaVoltSecondsSquaredPerMeter = 0.0389;
  public static final double kPDriveVel = 0.085;
  public static final double kTrackwidthMeters = 0.142072613;
  public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
  // autonomous constants
  public static final double kMaxSpeedMetersPerSecond = 0.8;
  public static final double kMaxAccelerationMetersPerSecondSquared = 0.8;
  // reasonable baseline values for a RAMSETE follower in units of meters and seconds
  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;

  // The robot's subsystems and commands are defined here...
  private final XboxController m_controller;
  private final RomiDrivetrainSubsystem m_romiDrivetrain;
  // Romi OnBoardIO
  private final OnBoardIO m_onBoardIO;
  // Create SmartDashboard chooser for autonomous routines
  private final SendableChooser<Command> m_chooser;
    
  // NOTE: The I/O pin functionality of the 5 exposed I/O pins depends on the hardware "overlay"
  // that is specified when launching the wpilib-ws server on the Romi raspberry pi.
  // By default, the following are available (listed in order from inside of the board to outside):
  // - DIO 8 (mapped to Arduino pin 11, closest to the inside of the board)
  // - Analog In 0 (mapped to Analog Channel 6 / Arduino Pin 4)
  // - Analog In 1 (mapped to Analog Channel 2 / Arduino Pin 20)
  // - PWM 2 (mapped to Arduino Pin 21)
  // - PWM 3 (mapped to Arduino Pin 22)
  //
  // Your subsystem configuration should take the overlays into account

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // initialize controller objects
    m_controller = new XboxController(0);
    // initialize subsystems objects
    m_romiDrivetrain = new RomiDrivetrainSubsystem();
    // initialize commands objects
    
    // Romi OnBoardIO
    m_onBoardIO = new OnBoardIO(ChannelMode.INPUT, ChannelMode.INPUT);
    // Create SmartDashboard chooser for autonomous routines
    m_chooser = new SendableChooser<>();
    
    // set default command for all subsystems
    m_romiDrivetrain.setDefaultCommand(getArcadeDriveCommand());

    // Configure the button bindings
    configureButtonBindings();

    // create Autonomous Command
    generateRamseteCommand();
    
    // Setup SmartDashboard options
    m_chooser.setDefaultOption("Ramsete Trajectory", generateRamseteCommand());
    m_chooser.addOption("Auto Routine Distance", new CG_AutoDistance(m_romiDrivetrain));
    m_chooser.addOption("Auto Routine Time", new CG_AutoTime(m_romiDrivetrain));

    SmartDashboard.putData(m_chooser);
  }


  /**
   * Generate a trajectory following Ramsete command
   *
   * This is very similar to the WPILib RamseteCommand example. It uses
   * constants defined in the Constants.java file. These constants were
   * found empirically by using the frc-characterization tool.
   *
   * @return A SequentialCommand that sets up and executes a trajectory following Ramsete command
   */
  private Command generateRamseteCommand() {
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter),
            kDriveKinematics, 10);

    TrajectoryConfig config =
        new TrajectoryConfig(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(kDriveKinematics)
                .addConstraint(autoVoltageConstraint);
    
    // This trajectory can be modified to suit your purposes
    // Note that all coordinates are in meters, and follow NWU conventions.
    // If you would like to specify coordinates in inches (which might be easier
    // to deal with for the Romi), you can use the Units.inchesToMeters() method
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            new Translation2d(0.5, 0.25),
            new Translation2d(1.0, -0.25),
            new Translation2d(1.5, 0)
        ),
        new Pose2d(0.0, 0, new Rotation2d(Math.PI)),
        config);

    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        m_romiDrivetrain::getPose,
        new RamseteController(kRamseteB, kRamseteZeta),
        new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter),
        kDriveKinematics,
        m_romiDrivetrain::getWheelSpeeds,
        new PIDController(kPDriveVel, 0, 0),
        new PIDController(kPDriveVel, 0, 0),
        m_romiDrivetrain::tankDriveVolts,
        m_romiDrivetrain);

    m_romiDrivetrain.resetOdometry(exampleTrajectory.getInitialPose());

    // Set up a sequence of commands
    // First, we want to reset the drivetrain odometry
    return new InstantCommand(() -> m_romiDrivetrain.resetOdometry(exampleTrajectory.getInitialPose()), m_romiDrivetrain)
        // next, we run the actual ramsete command
        .andThen(ramseteCommand)

        // Finally, we make sure that the robot stops
        .andThen(new InstantCommand(() -> m_romiDrivetrain.tankDriveVolts(0, 0), m_romiDrivetrain));
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    Button onBoardButtonA = new Button(m_onBoardIO::getButtonAPressed);
    onBoardButtonA
        .whenActive(new PrintCommand("Romi Buton A Pressed"))
        .whenInactive(new PrintCommand("Romi Buton A Released"));
    Button onBoardButtonB = new Button(m_onBoardIO::getButtonBPressed);
    onBoardButtonB
        .whenActive(new PrintCommand("Romi Buton B Pressed"))
        .whenInactive(new PrintCommand("Romi Buton B Released"));
    Button onBoardButtonC = new Button(m_onBoardIO::getButtonCPressed);
    onBoardButtonC
        .whenActive(new PrintCommand("Romi Buton C Pressed"))
        .whenInactive(new PrintCommand("Romi Buton C Released"));
    
    new Button(m_controller::getAButton).whenActive(new DriveDistance(0.5, 10, m_romiDrivetrain));
    new Button(m_controller::getBButton).whenActive(new DriveDistance(-0.5, 10, m_romiDrivetrain));
    new Button(m_controller::getXButton).whenActive(new TurnDegrees(0.5, 90, m_romiDrivetrain));
    new Button(m_controller::getYButton).whenActive(new TurnDegrees(-0.5, 90, m_romiDrivetrain));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  public Command getArcadeDriveCommand() {
    return new ArcadeDriveCommand(    // 1 is Left-Y-Axis; negate stick-Y-axis   // 4 is Right-X-Axis
      m_romiDrivetrain, () -> -m_controller.getRawAxis(1),  () -> m_controller.getRawAxis(4) * 0.8);
  }
}
