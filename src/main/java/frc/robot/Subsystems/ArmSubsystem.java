// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.PIDConstants;

public class ArmSubsystem extends SubsystemBase {

  private CANSparkMax arm_leftMotor = new CANSparkMax(ArmConstants.arm_leftMotor_PORT, MotorType.kBrushless);
  private CANSparkMax arm_rightMotor = new CANSparkMax(ArmConstants.arm_rightMotor_PORT, MotorType.kBrushless);

  private final DutyCycleEncoder arm_Encoder = new DutyCycleEncoder(0);

  private PIDController pid = new PIDController(PIDConstants.kP, PIDConstants.kI, PIDConstants.kD);

  private double processVar;
  private double Setpoint;

  private double Amplifier;

  public ArmSubsystem() {
    //arm_leftMotor.follow(arm_rightMotor);
    arm_leftMotor.setSmartCurrentLimit(60);
    arm_rightMotor.setSmartCurrentLimit(60);

    //pid.setSetpoint(0.001);
  }


  @Override
  public void periodic() {
    super.periodic();
    processVar = pid.calculate(arm_Encoder.getAbsolutePosition() - ArmConstants.kEncoderError);

    //arm_rightMotor.set(-processVar);
    //arm_leftMotor.set(-processVar);
    if(Setpoint == 0.001 && arm_Encoder.getAbsolutePosition() - ArmConstants.kEncoderError < 0.01){ //Cuando este abajo, deja de hacer crunchy
      Amplifier = 0;
    }
    else if(arm_Encoder.getAbsolutePosition() - ArmConstants.kEncoderError < 0.004){ //Cuando este abajo, deja de hacer crunchy
      Amplifier = 1;
    }

    arm_rightMotor.set(-processVar*Amplifier);
    arm_leftMotor.set(-processVar*Amplifier);

    SmartDashboard.putNumber("Arm Setpoint: ", pid.getSetpoint());
    SmartDashboard.putNumber("Arm AbsEncoder: ", arm_Encoder.getAbsolutePosition());
    SmartDashboard.putNumber("Arm Position: ", arm_Encoder.getAbsolutePosition() - ArmConstants.kEncoderError);
    SmartDashboard.putNumber("Arm ProcessVar: ", processVar);
    SmartDashboard.putBoolean("Arm Ready", atSetpoint());
    SmartDashboard.putNumber(" L Motor current", arm_leftMotor.getOutputCurrent());
    SmartDashboard.putNumber("R Motor current", arm_rightMotor.getOutputCurrent());
  }

  public Command setSetpoint(double Setpoint) {
    return runOnce(
      () -> pid.setSetpoint(Setpoint)
    );
  }

    public Command setAmplifierSetpoint(double Setpoint) {
      return runOnce(
        () -> {
          if(Setpoint == 0.262){ // mandar arriba
          Amplifier = 1;
          pid.setSetpoint(Setpoint);
        }
          else if(Setpoint == 0.106){ //mandar a tirar
          Amplifier = 1.20;
            pid.setSetpoint(Setpoint);
          }
          else if(Setpoint == 0.001){ // mandar a chupar
            Amplifier = 1.08;
            pid.setSetpoint(Setpoint);
          }
          else if(Setpoint == 0.150){ // mandar a tirar de lejos 
            Amplifier = 1.05;
            pid.setSetpoint(Setpoint);
          }
        
        }
      );
    }

    public Command autoSetAmplifierSetpoint(double Setpoint) {
      return runOnce(
        () -> {
          if(Setpoint == 0.2615){ // mandar arriba
          Amplifier = 1;
          pid.setSetpoint(Setpoint);
        }
          else if(Setpoint == 0.106){ //mandar a tirar
          Amplifier = 1.850;
            pid.setSetpoint(Setpoint);
          }
          else if(Setpoint == 0.001){ // mandar a chupar
            Amplifier = 1.7;
            pid.setSetpoint(Setpoint);
          }
          else if(Setpoint == 0.110){ // mandar a tirar de lejos 
            Amplifier = 1.750;
            pid.setSetpoint(Setpoint);
          }
          else if(Setpoint == 0.140){ // mandar a tirar de lejos 
            Amplifier = 1.22;
            pid.setSetpoint(Setpoint);
          }
        }
      );
    }
    public Command fourNote_autoSetAmplifierSetpoint(double Setpoint) {
      return runOnce(
        () -> {
          if(Setpoint == 0.2615){ // MANDAR ARRIBA 90
          Amplifier = 1;
          pid.setSetpoint(Setpoint);
        }
          else if(Setpoint == 0.1042){ //TIRO 0
          Amplifier = 1.990;
            pid.setSetpoint(Setpoint);
          }
          else if(Setpoint == 0.001){ // INTAKE mandar a chupar
            Amplifier = 2.0;
            pid.setSetpoint(Setpoint);
          }
          else if(Setpoint == 0.1335){ // SEGUNDO TIRO
            Amplifier = 2.15;
            pid.setSetpoint(Setpoint);
          }
          else if(Setpoint == 0.1320){ // mandar a tirar de lejos TERCER TIRO
            Amplifier = 2.30;
            pid.setSetpoint(Setpoint);
          }
        }
      );
    }
  
  public double getSetpoint(){
    return pid.getSetpoint();

  }

  public Command setAprilSetpoint(DoubleSupplier Area) {
    return runOnce(() -> {
      if (Area.getAsDouble() > 0.29) {
        Amplifier = 1.00001;
        pid.setSetpoint((-0.0935235516149 * Area.getAsDouble() + 0.1664));
      } else {
        Amplifier = 0.76;
        pid.setSetpoint((-0.0935235516149 * Area.getAsDouble() + 0.1677));
      }
    });
  }

  public Command autoSetSetpoint(double Setpoint) {
    return runOnce(() -> pid.setSetpoint(Setpoint));
  }

   public void manualSetpointFront() {
      Setpoint = pid.getSetpoint() + 0.01;
      pid.setSetpoint(Setpoint);
    }
    
    public void manualSetpointBack(){
      Setpoint = pid.getSetpoint() - 0.01;
      pid.setSetpoint(Setpoint);

    }
   
    public Boolean isUp(){
      if(arm_Encoder.getAbsolutePosition() > 0.05 && arm_Encoder.getAbsolutePosition() < 0.62){
        return true;
      }
      else{
        return false;
      }
    }

    public Boolean autoRunMode(){
      if((arm_Encoder.getAbsolutePosition() - ArmConstants.kEncoderError) < 0.02){
        return true;
      }
      else{
        return false;
      }
    }

    public Boolean isOnFront(){
      if(arm_Encoder.getAbsolutePosition() < 0.47){
        return true;
      }
      else{
        return false;
      }
    }

    
    public void setpointStop(){
      Setpoint = arm_Encoder.getAbsolutePosition() - ArmConstants.kEncoderError;
      pid.setSetpoint(Setpoint);
    }

    public Boolean atSetpoint(){
      if(((arm_Encoder.getAbsolutePosition() - ArmConstants.kEncoderError) > (getSetpoint()-ArmConstants.kAtSetpointTolerance)) &&
      ((arm_Encoder.getAbsolutePosition() - ArmConstants.kEncoderError) < (getSetpoint()+ArmConstants.kAtSetpointTolerance))){
        return true;
      }
      else{
        return false;
      }
    }
    public Boolean atSetpointBelowSpeaker(){
      if(((arm_Encoder.getAbsolutePosition() - ArmConstants.kEncoderError) > (getSetpoint()-ArmConstants.kAtSetpointBelowSpeakerTolerance)) &&
      ((arm_Encoder.getAbsolutePosition() - ArmConstants.kEncoderError) < (getSetpoint()+ArmConstants.kAtSetpointBelowSpeakerTolerance))){
        return true;
      }
      else{
        return false;
      }
    }



    
  }

