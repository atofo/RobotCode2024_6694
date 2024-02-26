// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutonomoDrivetrain;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.PIDConstants;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;

public class DrivetrainSubsystem extends SubsystemBase {

  private CANSparkMax leftFrontMotor = new CANSparkMax(DrivetrainConstants.leftFrontMotor_PORT, MotorType.kBrushless);
  private CANSparkMax rightFrontMotor = new CANSparkMax(DrivetrainConstants.rightFrontMotor_PORT,MotorType.kBrushless);
  private CANSparkMax leftRearMotor = new CANSparkMax(DrivetrainConstants.leftRearMotor_PORT, MotorType.kBrushless);
  private CANSparkMax rightRearMotor = new CANSparkMax(DrivetrainConstants.rightRearMotor_PORT, MotorType.kBrushless);

  //encoders
  /* 
  private DutyCycleEncoder leftFrontEncoder = new DutyCycleEncoder(1);
  private DutyCycleEncoder rightFrontEncoder = new DutyCycleEncoder(2);
  private DutyCycleEncoder leftRearEncoder = new DutyCycleEncoder(3);
  private DutyCycleEncoder rightRearEncoder1 = new DutyCycleEncoder(4);
  */
  
  
  private RelativeEncoder leftFrontEncoder = leftFrontMotor.getEncoder();
  private RelativeEncoder rightFrontEncoder = rightFrontMotor.getEncoder();
  private RelativeEncoder leftRearEncoder = leftRearMotor.getEncoder();
  private RelativeEncoder rightRearEncoder = rightFrontMotor.getEncoder();

  Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  Rotation2d m_Rotation2d = new Rotation2d(m_gyroscope.getAngle(IMUAxis.kYaw));

  MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(
  m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
);
  
;
  
  //PIDControlelr
  //PIDController pid = new PIDController(PIDConstants.kP, PIDConstants.kI, PIDConstants.kD);
  
  //processVars
  /* 
  private double leftFrontProcessVar;
  private double leftRearProcessVar;
  private double rightFrontProcessVar;
  private double rightRearProcessVar;
  */


  //Gyroscope
  public final static ADIS16470_IMU m_gyroscope = new ADIS16470_IMU();

  //Odometry
  private final MecanumDriveOdometry m_odometry;

  //MecanumDrive
  MecanumDrive m_drive = new MecanumDrive(
    leftFrontMotor::set, 
    leftRearMotor::set, 
    rightFrontMotor::set,
    rightRearMotor::set);


  /*
   * private final SysIdRoutine m_sysIdRoutine =
   * new SysIdRoutine(
   * new SysIdRoutine.Config(),
   * new SysIdRoutine.Mechanism(
   * (volts) -> {
   * leftFrontMotor.setVoltage(volts.in(Volts));
   * rightFrontMotor.setVoltage(volts.in(Volts));
   * leftRearMotor.setVoltage(volts.in(Volts));
   * rightRearMotor.setVoltage(volts.in(Volts));
   * },
   * null, // No log consumer, since data is recorded by URCL
   * this
   * )
   * );
   */

  public DrivetrainSubsystem() {
    //RestoreFactoryDefaults
    leftFrontMotor.restoreFactoryDefaults();  
    leftRearMotor.restoreFactoryDefaults();  
    rightFrontMotor.restoreFactoryDefaults();  
    rightRearMotor.restoreFactoryDefaults();  

    //BurnFlash
    leftFrontMotor.burnFlash();  
    leftRearMotor.burnFlash();  
    rightFrontMotor.burnFlash();  
    rightRearMotor.burnFlash();  

    //SetInverted
    leftFrontMotor.setInverted(true);
    leftRearMotor.setInverted(true);

    //Odometry
    m_odometry = new MecanumDriveOdometry(m_kinematics, m_Rotation2d, 
    new MecanumDriveWheelPositions(
      leftFrontEncoder.getPosition(),
      rightFrontEncoder.getPosition(),
      leftRearEncoder.getPosition(),
      rightRearEncoder.getPosition()
      ));

    m_odometry.resetPosition(m_Rotation2d, new MecanumDriveWheelPositions(
      leftFrontEncoder.getPosition(),
      rightFrontEncoder.getPosition(),
      leftRearEncoder.getPosition(),
      rightRearEncoder.getPosition()
      ), new Pose2d());

      //SetPositionConversionFactor
      leftFrontEncoder.setPositionConversionFactor(AutonomoDrivetrain.kLinerDistanceConversionFactor);
      leftRearEncoder.setPositionConversionFactor(AutonomoDrivetrain.kLinerDistanceConversionFactor);
      rightFrontEncoder.setPositionConversionFactor(AutonomoDrivetrain.kLinerDistanceConversionFactor);
      rightRearEncoder.setPositionConversionFactor(AutonomoDrivetrain.kLinerDistanceConversionFactor);

      //SetVelocityConversionFactor
      leftFrontEncoder.setVelocityConversionFactor(AutonomoDrivetrain.kLinerDistanceConversionFactor/60);
      leftRearEncoder.setVelocityConversionFactor(AutonomoDrivetrain.kLinerDistanceConversionFactor/60);
      rightFrontEncoder.setVelocityConversionFactor(AutonomoDrivetrain.kLinerDistanceConversionFactor/60);
      rightRearEncoder.setVelocityConversionFactor(AutonomoDrivetrain.kLinerDistanceConversionFactor/60);

      //Gyroscope
      m_gyroscope.reset();
      m_gyroscope.calibrate();


  }
/* 
  public void zeroHeading(){
    return m;
  }

  public void resetEncoders(){
    leftFrontEncoder.setPosition(0);
    leftRearEncoder.setPosition(0);
    rightFrontEncoder.setPosition(0);
    rightRearEncoder.setPosition(0);

  }

  public double getLeftFrontEncoderPosition(){
    return leftFrontEncoder.getPosition();
  }

  public double getLeftRearEncoderPosition(){
    return leftRearEncoder.getPosition();
  }

  public double getRightFrontEncoderPosition(){
    return rightFrontEncoder.getPosition();
  }

  public double getRightRearEncoderPosition(){
    return rightRearEncoder.getPosition();
  }

  public double getLeftFrontEncoderVelocity(){
    return leftFrontEncoder.getVelocity();
  }

  public double getLeftRearEncoderVelocity(){
    return leftRearEncoder.getVelocity();
  }

  public double getRightFrontEncoderVelocity(){
    return rightFrontEncoder.getVelocity();
  }

  public double getRightRearEncoderVelocity(){
    return rightRearEncoder.getVelocity();
  }

  public  double getHeading(){
    return m_gyroscope.getAngle(IMUAxis.kYaw);
  }

  public double getTurnRate(){
    return -m_gyroscope.getRate(IMUAxis.kYaw);
  }

  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose){
    resetEncoders();
    m_odometry.resetPosition(m_Rotation2d, new MecanumDriveWheelPositions(
      leftFrontEncoder.getPosition(),
      rightFrontEncoder.getPosition(),
      leftRearEncoder.getPosition(),
      rightRearEncoder.getPosition()), pose);
  }
  */

  public void drive(DoubleSupplier joystickX, DoubleSupplier joystickY, DoubleSupplier joystickZ,
      DoubleSupplier rightTrigger, DoubleSupplier leftTrigger) {

      if (rightTrigger.getAsDouble() > 0.1) {
            if (Math.abs(joystickX.getAsDouble()) < 0.1 &&
                Math.abs(joystickY.getAsDouble()) < 0.1 && 
                Math.abs(joystickZ.getAsDouble()) < 0.1 && 
                Math.abs(rightTrigger.getAsDouble()) < 0.1 && 
                Math.abs(leftTrigger.getAsDouble()) < 0.1) {
              m_drive.driveCartesian(0, 0, 0);
            } else {
              m_drive.driveCartesian(joystickZ.getAsDouble(), joystickY.getAsDouble(), rightTrigger.getAsDouble());
            }
          }
      
          else {
            if (Math.abs(joystickX.getAsDouble()) < 0.1 && 
                Math.abs(joystickY.getAsDouble()) < 0.1 && 
                Math.abs(joystickZ.getAsDouble()) < 0.1 && 
                Math.abs(rightTrigger.getAsDouble()) < 0.1 && 
                Math.abs(leftTrigger.getAsDouble()) < 0.1) {
              m_drive.driveCartesian(0, 0, 0);
            } else {
              m_drive.driveCartesian(joystickZ.getAsDouble(), joystickY.getAsDouble(), -leftTrigger.getAsDouble());
            }
          }
      }
  

  public void driveInverted(DoubleSupplier joystickX, DoubleSupplier joystickY, DoubleSupplier joystickZ,
      DoubleSupplier rightTrigger, DoubleSupplier leftTrigger) {

     if (rightTrigger.getAsDouble() > 0.1) {
            if (Math.abs(joystickX.getAsDouble()) < 0.1 && 
                 Math.abs(joystickY.getAsDouble()) < 0.1 &&
                Math.abs(joystickZ.getAsDouble()) < 0.1 && 
                Math.abs(rightTrigger.getAsDouble()) < 0.1 && 
                Math.abs(leftTrigger.getAsDouble()) < 0.1) {
              m_drive.driveCartesian(0, 0, 0);
            } else {
              m_drive.driveCartesian(-joystickZ.getAsDouble()  , -joystickY.getAsDouble() , -rightTrigger.getAsDouble());
            }
          }
      
          else {
            if (Math.abs(joystickX.getAsDouble()) < 0.1 && 
                Math.abs(joystickY.getAsDouble()) < 0.1 &&
                Math.abs(joystickZ.getAsDouble()) < 0.1 && 
                Math.abs(rightTrigger.getAsDouble()) < 0.1 && 
                Math.abs(leftTrigger.getAsDouble()) < 0.1) {
              m_drive.driveCartesian(0, 0, 0);
            } else {
              m_drive.driveCartesian(-joystickZ.getAsDouble() , -joystickX.getAsDouble(), leftTrigger.getAsDouble() );
            }
          }

    }

    

  /*
   * public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
   * return m_sysIdRoutine.quasistatic(direction);
   * }
   * 
   * public Command sysIdDynamic(SysIdRoutine.Direction direction) {
   * return m_sysIdRoutine.dynamic(direction);
   * }
   */

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("YAxis", m_gyroscope.getAngle(IMUAxis.kYaw));
    
    /* 
    leftFrontProcessVar = pid.calculate(leftFrontEncoder.getAbsolutePosition());
    leftRearProcessVar = pid.calculate(leftRearEncoder.getAbsolutePosition());
    rightFrontProcessVar = pid.calculate(rightFrontEncoder.getAbsolutePosition());
    rightRearProcessVar = pid.calculate(rightRearEncoder.getAbsolutePosition());
    */
    /* 
    leftFrontMotor.set(-leftFrontProcessVar * 0.8);  
    leftRearMotor.set(-leftRearProcessVar * 0.8);
    rightFrontMotor.set(-rightFrontProcessVar * 0.8);
    rightRearMotor.set(-rightRearProcessVar * 0.8);
    
    SmartDashboard.putNumber("Gyroscope", getHeading());
    SmartDashboard.putNumber("LeftFrontEncoder", getLeftFrontEncoderPosition());
    SmartDashboard.putNumber("LeftRearEncoder", getLeftRearEncoderPosition());
    SmartDashboard.putNumber("RightFrontEncoder", getRightFrontEncoderPosition());
    SmartDashboard.putNumber("RightRearEncoder", getRightRearEncoderPosition());
    */
    m_odometry.update(m_Rotation2d, new MecanumDriveWheelPositions(
      leftFrontEncoder.getPosition(),
      rightFrontEncoder.getPosition(),
      leftRearEncoder.getPosition(),
      rightRearEncoder.getPosition()));
  }

  public void driveAutonomo(){
    if (m_gyroscope.getAngle(IMUAxis.kYaw) < 40) {
      m_drive.driveCartesian(0.15, 0, 0);
    } 
    if (m_gyroscope.getAngle(IMUAxis.kYaw) > 50) {
      m_drive.driveCartesian(-0.15, 0, 0);
    }
      //pid.setSetpoint(pid.getSetpoint() + 10);
    }

  public Command driveAutonomo2(double angleInput){
    double error = (angleInput - m_gyroscope.getAngle(IMUAxis.kYaw))/angleInput;
    return runOnce(() -> m_drive.driveCartesian(error, 0, 0));

  }
}
