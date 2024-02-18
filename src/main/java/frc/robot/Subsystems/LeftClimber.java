// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class LeftClimber extends SubsystemBase {
  /** Creates a new LeftClimber. */
  private CANSparkMax m_LeftClimber = new CANSparkMax(ClimberConstants.climber_leftMotor_PORT, MotorType.kBrushless);
  public LeftClimber() {
    m_LeftClimber.setInverted(true);
  }

  public void leftClimberUp(){
    m_LeftClimber.set(0.3);
  }

  public void leftClimberDown(){
    m_LeftClimber.set(-0.3);
  }

  public void leftClimberOFF(){
    m_LeftClimber.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
