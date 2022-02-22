// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.romi.RomiGyro;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RomiDrivetrainSubsystem extends SubsystemBase {
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterInch = 2.75591; // 70 mm

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  
  // Set up the RomiGyro
  private final RomiGyro m_gyro;

  // Set up the BuiltInAccelerometer
  @SuppressWarnings("unused")
  private final BuiltInAccelerometer m_accelerometer; 

  // ChasisSpeeds class for tracking robot speeds
  @SuppressWarnings("unused")
  private final ChassisSpeeds m_chassisSpeeds;

  // Odometry class for tracking robot pose
  @SuppressWarnings("unused")
  private final DifferentialDriveOdometry m_odometry;

  // Also show a field diagram on SmartDashboard
  private final Field2d m_field2d;

  /** Creates a new RomiDrivetrain. */
  public RomiDrivetrainSubsystem() {
    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);

    // Invert right side since motor is flipped
    m_leftMotor.setInverted(false); 
    m_rightMotor.setInverted(true);

    // set up odometry 
    m_gyro = new RomiGyro();
    m_accelerometer = new BuiltInAccelerometer(); 
    m_chassisSpeeds = new ChassisSpeeds();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

    resetEncoders();

    m_field2d = new Field2d();
    SmartDashboard.putData("field", m_field2d);
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public double getLeftDistanceInch() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceInch() {
    return m_rightEncoder.getDistance();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
