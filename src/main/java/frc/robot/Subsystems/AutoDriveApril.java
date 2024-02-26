/* // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
//import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoDriveApril extends SubsystemBase {

  public final Limelight limelight;
  public final PhotonCamera photonCamera;
  ADIS16470_IMU Gyroscope = new ADIS16470_IMU();

  int Id;

  double yaw;
  double pitch;
  double area;
  double skew;

  double DesiredArea = 0;
  double DesiredYaw = 0;
  double DesiredAngle = 0;

  double XSpeed;
  double YSpeed;
  double ZSpeed;

  public AutoDriveApril() {

    limelight = new Limelight();

    photonCamera = new PhotonCamera("photonvision");

    Id = 0;
    yaw = 0;
    pitch = 0;
    area = 0;
    skew = 0;

  }

  @Override
  public void periodic() {
    var result = photonCamera.getLatestResult();
    boolean hasTargets = result.hasTargets();

    PhotonTrackedTarget target;

    if (hasTargets == true) {

      target = result.getBestTarget();

      Id = target.getFiducialId();
      yaw = target.getYaw();
      pitch = target.getPitch();
      area = target.getArea();
      skew = target.getSkew();

    } else {

      Id = 0;
      yaw = 0;
      pitch = 0;
      area = 0;
      skew = 0;

    }
    SmartDashboard.putNumber("Gyroscope", Gyroscope.getAngle(IMUAxis.kYaw));

  }

  /*
   * public double XSpeed() {
   * if (limelight.yaw > 1) {
   * return 0.2;
   * } else if (yaw < -1) {
   * return -0.2;
   * } else {
   * return 0;
   * }
   * }
   * 
   * public double YSpeed() {
   * if (limelight.area < 8.20 && area > 0) {
   * return 0.1;
   * } else if (area > 7 && area > 0) {
   * return -0.1;
   * } else {
   * return 0;
   * }
   * }
   * 
   * public double ZSpeed() {
   * 
   * if (Gyroscope.getAngle(IMUAxis.kYaw) < -4.5) {
   * return -(0.25 + (Gyroscope.getAngle(IMUAxis.kYaw) * 0.005));
   * } else if (Gyroscope.getAngle(IMUAxis.kYaw) > 4.5) {
   * return 0.25 + (Gyroscope.getAngle(IMUAxis.kYaw) * 0.005);
   * } else {
   * return 0;
   * }
   * 
   * }
   */

/* ublic Command DriveAuto() {

  return run(() ->

  driveTrain.drive(() -> YSpeed, () -> XSpeed, () -> ZSpeed));

} 

public void DriveAuto(DoubleSupplier DesiredYaw, DoubleSupplier DesiredArea, DoubleSupplier DesiredAngle){

  if (Gyroscope.getAngle(IMUAxis.kYaw) < DesiredAngle.getAsDouble() - 4) {
    ZSpeed = -(0.25 + (Gyroscope.getAngle(IMUAxis.kYaw) * 0.005));
  } else if (Gyroscope.getAngle(IMUAxis.kYaw) > DesiredAngle.getAsDouble() + 4) {
    ZSpeed = 0.25 + (Gyroscope.getAngle(IMUAxis.kYaw) * 0.005);
  } else {
    ZSpeed = 0;
  }


  XSpeed = 0;
  YSpeed = 0;

  if (area < DesiredArea.getAsDouble() - 1 && area > 0) {
    YSpeed = -0.15;
  } else if (area > DesiredArea.getAsDouble() + 1 && area > 0) {
    YSpeed = 0.15;
  } else {
    YSpeed = 0;
  }

  if (yaw > DesiredYaw.getAsDouble() + 1) {
    XSpeed = -0.2;
  } else if (yaw < DesiredYaw.getAsDouble() - 1) {
    XSpeed = 0.2;
  } else {
    XSpeed = 0;
  }
} 
public double XSpeed(DoubleSupplier DesiredYaw) {
  
  if (limelight.Yaw() > DesiredYaw.getAsDouble() + 1) {
  return 0.2;
  } else if (limelight.Yaw() < DesiredYaw.getAsDouble() - 1) {
  return -0.2;
  } else {
  return 0;
  }
  }
  
  public double YSpeed(DoubleSupplier DesiredArea) {
  if (limelight.Area() < DesiredArea.getAsDouble() + 1 && area > 0) {
  return 0.1;
  } else if (limelight.Area() > DesiredArea.getAsDouble() - 1 && area > 0) {
  return -0.1;
  } else {
  return 0;
  }
  }
  
  public double ZSpeed(DoubleSupplier DesiredAngle) {
  
  if (Gyroscope.getAngle(IMUAxis.kYaw) < DesiredAngle.getAsDouble() - 4) {
  return -(0.25 + (Gyroscope.getAngle(IMUAxis.kYaw) * 0.005));
  } else if (Gyroscope.getAngle(IMUAxis.kYaw) > DesiredAngle.getAsDouble() + 4) {
  return 0.25 + (Gyroscope.getAngle(IMUAxis.kYaw) * 0.005);
  } else {
  return 0;
  }
  
  }*/

/*
 * public Command DriveAuto(){
 * 
 * return run( () ->
 * 
 * mecanumDrive.driveCartesian(YSpeed, XSpeed, ZSpeed));
 * 
 * }
 

} */

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

//import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoDriveApril extends SubsystemBase {



  public final Limelight limelight;

  private Gyro Gyroscope;

  public AutoDriveApril() {

    limelight = new Limelight();


    Gyroscope = new Gyro();

  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Gyroscope", Gyroscope.getZ());
    SmartDashboard.putNumber("Yaw()", limelight.Yaw());
    SmartDashboard.putNumber("Area()", limelight.Area());

  }

  public double XSpeed(DoubleSupplier DYaw) {
    if (limelight.Yaw() > DYaw.getAsDouble() + 1) { //Rango de error para VelocidadX ((SOLO CAMBIAR DYaw)
      return -0.2;
    } else if (limelight.Yaw() < DYaw.getAsDouble() - 1) {
      return 0.2;
    } else {
      return 0;
    }
  }

  public double YSpeed(DoubleSupplier DArea) {
    if (limelight.Area() < DArea.getAsDouble() + 1 && limelight.Area() > 0) { //Rango de error para VelocidadY ((SOLO CAMBIAR DArea)
      return 0.15;
    } else if (limelight.Area() > DArea.getAsDouble() - 1 && limelight.Area() > 0) {
      return -0.15;
    }else{
      return 0;
    }
  }

  public double ZSpeed() {

    if (Gyroscope.getZ() <  -4) {//Declara un rango de error
      return 0.15 + (Gyroscope.getZ() * 0.005); //Regresa una velocidad relativa al angulo considerando 0 como setpoint
    } else if (Gyroscope.getZ() > 4) {//Declara un rango de error
      return -(0.15 + (Gyroscope.getZ() * 0.005)); //Regresa una velocidad relativa al angulo considerando 0 como setpoint invertido
    } else {
      return 0;
    }

  }
  public double GyroSpeed(DoubleSupplier DAngle) {

    if (Gyroscope.getZ() < DAngle.getAsDouble() - 4) { //Declara un rango de error
      return -(0.15 + ((Gyroscope.getZ() + (Gyroscope.getZ() - DAngle.getAsDouble())) * 0.001));  //Regresa una velocidad relativa al angulo considerando considerando DAngulo como setpoint invertido(no esta calado al cien, jugar con los signos y posicion de DAngle en esta linea)
    } else if (Gyroscope.getZ() > DAngle.getAsDouble() + 4) { //Declara un rango de error
      return 0.15 + ((Gyroscope.getZ() - (Gyroscope.getZ() - DAngle.getAsDouble())) * 0.001); //Regresa una velocidad relativa al angulo considerando DAngulo como setpoint (no esta calado al cien, jugar con los signos y posicion de DAngle en esta linea)
    } else {
      return 0;
    }

  }

  

  /*
   * public Command DriveAuto(DoubleSupplier DesiredYaw, DoubleSupplier
   * DesiredArea, DoubleSupplier DesiredAngle) {
   * 
   * DArea = DesiredYaw.getAsDouble();
   * DYaw = DesiredArea.getAsDouble();
   * DAngle = DesiredAngle.getAsDouble();
   * 
   * return run(() ->
   * 
   * driveTrain.drive(() -> XSpeed(), () -> YSpeed(), () -> ZSpeed()));
   * 
   * }
   */

}
