// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeLauncherConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private CANSparkMax m_downMotor = new CANSparkMax(IntakeLauncherConstants.intakelauncher_downMotor_PORT, MotorType.kBrushless);
  private SparkPIDController m_downPidController = m_downMotor.getPIDController();
  private RelativeEncoder m_downEncoder = m_downMotor.getEncoder();
  private double downSetPoint = ShooterConstants.maxRPM;
  
  
  private CANSparkMax m_upMotor = new CANSparkMax(IntakeLauncherConstants.intakelauncher_upMotor_PORT, MotorType.kBrushless);
  private SparkPIDController m_upPidController = m_upMotor.getPIDController();
  private RelativeEncoder m_upEncoder = m_upMotor.getEncoder();
  private double upSetPoint = ShooterConstants.maxRPM;

  public ShooterSubsystem() {
    m_upMotor.setInverted(true);
    upSetPoint =  0;
    downSetPoint = 0;

    m_upPidController.setP(ShooterConstants.kP);
    m_upPidController.setI(ShooterConstants.kI);
    m_upPidController.setD(ShooterConstants.kD);
    m_upPidController.setIZone(ShooterConstants.kIz);
    m_upPidController.setFF(ShooterConstants.kUpFF);
    m_upPidController.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);
    
    m_downPidController.setP(ShooterConstants.kP);
    m_downPidController.setI(ShooterConstants.kI);
    m_downPidController.setD(ShooterConstants.kD);
    m_downPidController.setIZone(ShooterConstants.kIz);
    m_downPidController.setFF(ShooterConstants.kDownFF);
    m_downPidController.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);

    m_upMotor.enableVoltageCompensation(11);
    m_downMotor.enableVoltageCompensation(11);
  }

  public Command autoEnable() {
    return runOnce(() -> {
      upSetPoint = (ShooterConstants.maxRPM);
      downSetPoint = (ShooterConstants.maxRPM);
    });
  }
  public Command autoDisable() {
    return runOnce(() -> {
      upSetPoint = (0);
      downSetPoint = (0);
    });
  }
  
  public void disable() {
      upSetPoint = (0);
      downSetPoint = (0);
  }
  public void enable() {
      upSetPoint = (ShooterConstants.maxRPM);
      downSetPoint = (ShooterConstants.maxRPM);
  }

  public boolean atSetpoint() {
    if((((Math.abs(m_upEncoder.getVelocity()) + Math.abs(m_downEncoder.getVelocity()))/2 > ShooterConstants.maxRPM - ShooterConstants.RPMmargin) &&
    ((Math.abs(m_upEncoder.getVelocity()) + Math.abs(m_downEncoder.getVelocity()))/2 < ShooterConstants.maxRPM + ShooterConstants.RPMmargin))){
      return true;
    }
    else{
      return false;
    }
  }

  public boolean charging(){
    if(upSetPoint > 1 && downSetPoint > 1){
      return true;
    }
    else{
      return false;
    }
  }

  public void emergencyShoot(){
  m_downMotor.set(1);
  m_upMotor.set(1);
  }
  
  public void emergencyStop(){
  m_downMotor.set(1);
  m_upMotor.set(1);
  }

  @Override
  public void periodic(){
    super.periodic();
    m_upPidController.setReference(upSetPoint, CANSparkMax.ControlType.kVelocity);
    m_downPidController.setReference(downSetPoint, CANSparkMax.ControlType.kVelocity);
    SmartDashboard.putNumber("SetPoint", upSetPoint);
    SmartDashboard.putNumber("Up Velocity", m_upEncoder.getVelocity());
    SmartDashboard.putNumber("Down Velocity", m_downEncoder.getVelocity());
    SmartDashboard.putBoolean("Shooter Ready", atSetpoint());
    
  }

}
