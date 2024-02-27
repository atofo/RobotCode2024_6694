 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private final PhotonCamera Limelight;

  private boolean hasTargets;
 
  private int Id;
 
  private double yaw;
  private double pitch;
  private double area;
  private double skew;

  public Limelight() {

    Limelight = new PhotonCamera("photonvision");

    Id = 0;
    yaw = 0;
    pitch = 0;
    area = 0;
    skew = 0;

  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Area", Area());
    SmartDashboard.putNumber("Yaw", Yaw());

    var result = Limelight.getLatestResult();
    boolean hasTargets = result.hasTargets();

    PhotonTrackedTarget target;

    Transform3d bestCameraToTarget;
    Transform3d alternateCameraToTarget;


    if (hasTargets == true) {

      target = result.getBestTarget();

      bestCameraToTarget = target.getBestCameraToTarget();
      alternateCameraToTarget = target.getAlternateCameraToTarget();
      

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
  }

  public int AprilId() {
    return Id;
  }

  public double Area() {
    return area;
  }

  public double Yaw() {
    return yaw;
  }

  public double Skew() {
    return skew;
  }

  public double Pitch() {
    return pitch;
  }

} 
 