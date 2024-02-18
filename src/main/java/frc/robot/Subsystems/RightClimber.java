// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class RightClimber extends SubsystemBase {
  /** Creates a new RightClimber. */
  private CANSparkMax m_RightClimber = new CANSparkMax(ClimberConstants.climber_rightMotor_PORT, MotorType.kBrushless);
  public RightClimber() {

  }

  public void rightClimberUp(){
    m_RightClimber.set(0.3);
  }

  public void rightClimberDown(){
    m_RightClimber.set(-0.3);
  }

  public void rightClimberOFF(){
    m_RightClimber.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
