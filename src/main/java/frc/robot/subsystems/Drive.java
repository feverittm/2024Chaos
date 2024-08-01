// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
  // The motors on the left side of the drive.
  private final WPI_VictorSPX m_leftMotor1 = new WPI_VictorSPX(DriveConstants.kLeftMotor1CANID);
  private final WPI_VictorSPX m_leftMotor2 = new WPI_VictorSPX(DriveConstants.kLeftMotor2CANID);

  // The motors on the right side of the drive.
  private final WPI_VictorSPX m_rightMotor1 = new WPI_VictorSPX(DriveConstants.kRightMotor1CANID);
  private final WPI_VictorSPX m_rightMotor2 = new WPI_VictorSPX(DriveConstants.kRightMotor2CANID);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotor1, m_rightMotor1);

  // The left-side drive encoder
  private final Encoder m_leftEncoder = new Encoder(
      DriveConstants.kLeftEncoderDIOs[0],
      DriveConstants.kLeftEncoderDIOs[1],
      DriveConstants.kLeftEncoderReversed);

  // The right-side drive encoder
  private final Encoder m_rightEncoder = new Encoder(
      DriveConstants.kRightEncoderDIOs[0],
      DriveConstants.kRightEncoderDIOs[1],
      DriveConstants.kRightEncoderReversed);

  /** Creates a new Drive subsystem. */
  public Drive() {
    m_leftMotor1.configFactoryDefault();
    m_leftMotor2.configFactoryDefault();
    m_rightMotor1.configFactoryDefault();
    m_rightMotor2.configFactoryDefault();

    m_leftMotor2.follow(m_leftMotor1);
    m_rightMotor2.follow(m_rightMotor1);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor1.setInverted(DriveConstants.INVERT_RIGHT);
    m_leftMotor1.setInverted(DriveConstants.INVERT_LEFT);
    m_leftMotor2.setInverted(InvertType.FollowMaster);
		m_rightMotor2.setInverted(InvertType.FollowMaster);


    // Sets the distance per pulse for the encoders
    m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
  }

    /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive (double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  public void setOutput(double left, double right) {
    m_leftMotor1.set(left);
    m_rightMotor1.set(right);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  
}
