// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;


import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;

public class DrivetrainSubsystem extends SubsystemBase {

  private CANSparkMax leftFrontMotor = new CANSparkMax(DrivetrainConstants.leftFrontMotor_PORT, MotorType.kBrushless);
  private CANSparkMax rightFrontMotor = new CANSparkMax(DrivetrainConstants.rightFrontMotor_PORT, MotorType.kBrushless);
  private CANSparkMax leftRearMotor = new CANSparkMax(DrivetrainConstants.leftRearMotor_PORT, MotorType.kBrushless);
  private CANSparkMax rightRearMotor = new CANSparkMax(DrivetrainConstants.rightRearMotor_PORT, MotorType.kBrushless);


  private ADIS16470_IMU Gyroscope = new ADIS16470_IMU();


  MecanumDrive m_drive = new MecanumDrive(leftFrontMotor::set, leftRearMotor::set, rightFrontMotor::set,
      rightRearMotor::set);


  //private RelativeEncoder leftFrontEncoder = leftFrontMotor.getEncoder();
  //private RelativeEncoder rightFrontEncoder = rightFrontMotor.getEncoder();
  private RelativeEncoder leftRearEncoder = leftRearMotor.getEncoder();
  private RelativeEncoder rightRearEncoder = rightFrontMotor.getEncoder();
  
  
  public DrivetrainSubsystem() {
    leftFrontMotor.restoreFactoryDefaults();
    rightFrontMotor.restoreFactoryDefaults();
    leftRearMotor.restoreFactoryDefaults();
    rightRearMotor.restoreFactoryDefaults();

    rightFrontMotor.setInverted(false);
    rightRearMotor.setInverted(true);

    leftFrontMotor.setInverted(false);
    leftRearMotor.setInverted(true);

    leftFrontMotor.burnFlash();
    rightFrontMotor.burnFlash();
    leftRearMotor.burnFlash();
    rightRearMotor.burnFlash();
    

    leftRearEncoder.setPosition(0);
    rightRearEncoder.setPosition(0);

    leftRearEncoder.setPositionConversionFactor(DrivetrainConstants.kEncoderConversionFactor);    //esto usa rotaciones y se multiplica por el argumento
    rightRearEncoder.setPositionConversionFactor(DrivetrainConstants.kEncoderConversionFactor);
    leftRearEncoder.setVelocityConversionFactor(DrivetrainConstants.kEncoderConversionFactor / 60);
    rightRearEncoder.setVelocityConversionFactor(DrivetrainConstants.kEncoderConversionFactor / 60);

    Gyroscope.calibrate();
  }

  // Encoders
  public void resetEncoders(){
    leftRearEncoder.setPosition(0);
    rightRearEncoder.setPosition(0);
  }

  public double getRightEncoderPosition(){
    return -rightRearEncoder.getPosition();
  }
  
  public double getLeftEncoderPosition(){
    return -leftRearEncoder.getPosition();
  }
  
  public double getRightEncoderVelocity(){
    return -rightRearEncoder.getVelocity();
  }
  
  public double getLeftEncoderVelocity(){
    return -leftRearEncoder.getVelocity();
  }

  public void tankDriveVolts(double leftVolts, double rightVolts){
    leftFrontMotor.setVoltage(leftVolts);
    rightFrontMotor.setVoltage(rightVolts);
    leftRearMotor.setVoltage(leftVolts);
    rightRearMotor.setVoltage(rightVolts);
    m_drive.feed();
  } 

  public double getAvarageEncoderDistance(){
    return ((getLeftEncoderPosition()+getRightEncoderPosition()) / 2);
  }






  public void drive(DoubleSupplier joystickX, DoubleSupplier joystickZ,
      DoubleSupplier rightTrigger, DoubleSupplier leftTrigger, boolean buttonToggle) {


    if(buttonToggle == true){
      if (rightTrigger.getAsDouble() > 0.1) {
            if (Math.abs(joystickX.getAsDouble()) < 0.1
                && Math.abs(joystickZ.getAsDouble()) < 0.1 && Math.abs(rightTrigger.getAsDouble()) < 0.1
                && Math.abs(leftTrigger.getAsDouble()) < 0.1) {
              m_drive.driveCartesian(0, 0, 0);
            } else {
              m_drive.driveCartesian(joystickZ.getAsDouble(), -joystickX.getAsDouble(), rightTrigger.getAsDouble());
            }
          }
      
          else {
            if (Math.abs(joystickX.getAsDouble()) < 0.1
                && Math.abs(joystickZ.getAsDouble()) < 0.1 && Math.abs(rightTrigger.getAsDouble()) < 0.1
                && Math.abs(leftTrigger.getAsDouble()) < 0.1) {
              m_drive.driveCartesian(0, 0, 0);
            } else {
              m_drive.driveCartesian(joystickZ.getAsDouble(), -joystickX.getAsDouble(), -leftTrigger.getAsDouble());
            }
          }
    }

    else{

      if (rightTrigger.getAsDouble() > 0.1) {
            if (Math.abs(joystickX.getAsDouble()) < 0.1
                && Math.abs(joystickZ.getAsDouble()) < 0.1 && Math.abs(rightTrigger.getAsDouble()) < 0.1
                && Math.abs(leftTrigger.getAsDouble()) < 0.1) {
              m_drive.driveCartesian(0, 0, 0);
            } else {
              m_drive.driveCartesian(-joystickZ.getAsDouble()  , joystickX.getAsDouble() , -rightTrigger.getAsDouble());
            }
          }
      
          else {
            if (Math.abs(joystickX.getAsDouble()) < 0.1
                && Math.abs(joystickZ.getAsDouble()) < 0.1 && Math.abs(rightTrigger.getAsDouble()) < 0.1
                && Math.abs(leftTrigger.getAsDouble()) < 0.1) {
              m_drive.driveCartesian(0, 0, 0);
            } else {
              m_drive.driveCartesian(-joystickZ.getAsDouble() , joystickX.getAsDouble(), leftTrigger.getAsDouble() );
            }
          }

    }

  }

  /* Returns a command that drives the robot forward a specified distance at a specified speed.
   
    @param distanceMeters The distance to drive forward in meters
    @param speed The fraction of max speed at which to drive

   */

  public Command driveDistanceCommand(double distanceMeters, double speed) {
    return runOnce(
        // Reset encoders at the start of the command
            () -> resetEncoders())
        // Drive forward at specified speed
        .andThen(run(() -> m_drive.driveCartesian(0, speed, 0)))
        // End command when we've traveled the specified distance
        .until(
            () ->
                Math.max(getLeftEncoderPosition(), getRightEncoderPosition())
                    >= distanceMeters)
        // Stop the drive when the command ends
        .finallyDo(interrupted -> m_drive.stopMotor());
    
      }
        

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("Posicion atras izquierdo: ", getLeftEncoderPosition());
    SmartDashboard.putNumber("Posicion atras derecho: ", getRightEncoderPosition());
    SmartDashboard.putNumber("Velocidad atras izquierdo: ", getLeftEncoderVelocity());
    SmartDashboard.putNumber("Velocidad atras derecho: ", getLeftEncoderVelocity());
    SmartDashboard.putNumber("Gyroscope", Gyroscope.getAngle(IMUAxis.kYaw));
  }
}
