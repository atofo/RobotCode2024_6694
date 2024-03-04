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
    m_LeftClimber.enableVoltageCompensation(11);
  }

  public void leftClimberUp(){
    m_LeftClimber.set(0.3);
  
/*     if(m_LeftEncoder.getPosition() > 170){
      m_LeftClimber.set(0);
    }
    else{
    }   */
  }

  public void leftClimberDown(){
    m_LeftClimber.set(-0.3);
/*     if(m_LeftEncoder.getPosition() < 2){
      m_LeftClimber.set(0);
    }
    else{

    }  */
  }

  public void leftClimberOFF(){
    m_LeftClimber.set(0);
  }


  public Boolean LeftisUp(){
    if(m_LeftEncoder.getPosition() >= 3 && m_LeftEncoder.getPosition() <= 168){
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
