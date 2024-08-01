// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.LauncherConstants;

public class Hood extends PIDSubsystem {
  /** Creates a new Hood. */
  private final WPI_TalonSRX hoodMotor = new WPI_TalonSRX(LauncherConstants.HOOD_MOTOR_CANID);

  private final DigitalInput hoodLimitSwitch = new DigitalInput(LauncherConstants.HOOD_LIMIT_SWITCH_DIO);

  private final Encoder hoodAngleEncoder = new Encoder(LauncherConstants.HOOD_ENCODER_DIO_CHANNEL[0],
      LauncherConstants.HOOD_ENCODER_DIO_CHANNEL[1], LauncherConstants.HOOD_ENCODER_INVERTED);

  public Hood() {
    super(
        // The PIDController used by the subsystem
        new PIDController(0, 0, 0));

    hoodMotor.configFactoryDefault();
    hoodMotor.setInverted(LauncherConstants.HOOD_MOTOR_INVERTED);
    hoodMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    // Resets the encoder to read a distance of zero
    hoodAngleEncoder.reset();
  }

  /**
   * @return
   */
  public boolean hoodLimitSwitchActivated() {
    SmartDashboard.putBoolean("Hood Limit Switch", hoodLimitSwitch.get());
    if (LauncherConstants.HOOD_LIMIT_SWITCH_TRUE_WHEN_ACTIVATED)
      return hoodLimitSwitch.get();
    else
      return !hoodLimitSwitch.get();
  }

  /**
   * @return the current position of the hood on the rack
   */
  public double getHoodPosition() {
    return hoodAngleEncoder.get() / LauncherConstants.HOOD_ENCODER_CPR;
  }

  /**
   * @param voltage
   */
  public void setHoodVoltage(double voltage) {
    double toApply;

    if (getHoodPosition() >= LauncherConstants.HOOD_MAX_ANGLE && voltage > 0) {
      toApply = 0; // if at or above max angle, cancel all + (up) voltages
    } else {
      if (!hoodLimitSwitch.get() && hoodAngleEncoder.get() <= 0 && voltage < 0) {
        toApply = 0;
      } else {
        toApply = voltage;
      }
    }

    hoodMotor.set(toApply);
  }

  @Override
  public void useOutput(double output, double setpoint) {
  //   // Use the output here
  //   double mset = getController().calculate(setpoint);
  //   //setHoodVoltage(output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return hoodAngleEncoder.getRaw() / LauncherConstants.HOOD_ENCODER_CPR;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Hood Limit Switch", hoodLimitSwitch.get());
    SmartDashboard.putNumber("Raw Hood Position:", hoodAngleEncoder.getRaw());
    SmartDashboard.putNumber("Basic Hood Position:", hoodAngleEncoder.getRaw());
    SmartDashboard.putNumber("Hood Setpoint", getController().getSetpoint());
  }
}
