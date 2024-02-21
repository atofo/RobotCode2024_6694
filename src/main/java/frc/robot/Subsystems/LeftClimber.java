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

public class LeftClimber extends SubsystemBase {
  /** Creates a new LeftClimber. */
  private CANSparkMax m_LeftClimber = new CANSparkMax(ClimberConstants.climber_leftMotor_PORT, MotorType.kBrushless);
  private RelativeEncoder m_LeftEncoder = m_LeftClimber.getEncoder();
  public LeftClimber() {
    m_LeftClimber.setInverted(true);
  }

  public void leftClimberUp(){
    m_LeftClimber.set(0.3);
    if(m_LeftEncoder.getPosition() > 68){
      m_LeftClimber.set(0);
    }
    else{
      m_LeftClimber.set(0.3);
    } 
  }

  public void leftClimberDown(){
if(m_LeftEncoder.getPosition() < 5){
      m_LeftClimber.set(0);
    }
    else{
      m_LeftClimber.set(-0.3);
    }
  }

  public void leftClimberOFF(){
    m_LeftClimber.set(0);
  }

  public void leftClimberEmergency(){
    if(m_LeftEncoder.getPosition() < 5){
          m_LeftClimber.set(0);
        }
        else{
          m_LeftClimber.set(-0.8);
        }
      }

  public Boolean LeftisUp(){
    if(m_LeftEncoder.getPosition() > 0.40 && m_LeftEncoder.getPosition() < 0.100){
      return true;
    }
      else{
      return false;
      }
    }
  
  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("LClimberEnc", m_LeftEncoder.getPosition());
    // This method will be called once per scheduler run
  }
}
