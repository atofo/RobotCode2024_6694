// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class RightClimber extends SubsystemBase {
  /** Creates a new RightClimber. */
  private CANSparkMax m_RightClimber = new CANSparkMax(ClimberConstants.climber_rightMotor_PORT, MotorType.kBrushless);
  private RelativeEncoder m_RightEncoder = m_RightClimber.getEncoder();

  public RightClimber() {
    m_RightClimber.enableVoltageCompensation(11);
  }

  public void rightClimberUp(){
    m_RightClimber.set(0.3);
/*     if(m_RightEncoder.getPosition() > 170){
      m_RightClimber.set(0);
    }
    else{
    }  */
  }

  public void rightClimberDown(){
    m_RightClimber.set(-0.3);
/*     if(m_RightEncoder.getPosition() < 2){
      m_RightClimber.set(0);
    }
    else{
    } */
  } 

  public void rightClimberOFF(){
    m_RightClimber.set(0);
  }

  public Boolean RightisUp(){
    if(m_RightEncoder.getPosition() >= 3 && m_RightEncoder.getPosition() <= 168){
      return true;
    }
      else{
      return false;
      }
    }

  @Override
  public void periodic() {
        super.periodic();
    SmartDashboard.putNumber("RClimberEnc", m_RightEncoder.getPosition());
    // This method will be called once per scheduler run
  }
}
