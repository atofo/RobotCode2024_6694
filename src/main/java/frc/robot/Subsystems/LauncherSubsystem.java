// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeLauncherConstants;
import frc.robot.Constants.PIDConstants;

public class LauncherSubsystem extends SubsystemBase {

  private CANSparkMax m_downMotor = new CANSparkMax(IntakeLauncherConstants.intakelauncher_downMotor_PORT, MotorType.kBrushless);
  private CANSparkMax m_upMotor = new CANSparkMax(IntakeLauncherConstants.intakelauncher_upMotor_PORT, MotorType.kBrushless);

/*   private Encoder launcher_encoder = new Encoder(4, 5); 

  PIDController pid = new PIDController(PIDConstants.kP, PIDConstants.kI, PIDConstants.kD);
 */
  

public LauncherSubsystem() {
    m_upMotor.restoreFactoryDefaults();
    m_downMotor.restoreFactoryDefaults();
    
    m_upMotor.burnFlash();
    m_downMotor.burnFlash();

    m_upMotor.setInverted(true);
  }

  public void throwNote(){
    m_downMotor.set(1);
    m_upMotor.set(1);
  }

  public void launcherOFF(){
      m_downMotor.set(0);
      m_upMotor.set(0);
  }

    public Command autoLaunchOn() {
    return runOnce(() -> {
    m_downMotor.set(1);
    m_upMotor.set(1);
    });
  }
    public Command autoLaunchOff() {
    return runOnce(() -> {
    m_downMotor.set(0);
    m_upMotor.set(0);
    });
  }

  @Override
  public void periodic() {
/*     launcher_encoder.getDistance();
    SmartDashboard.putNumber("Distancia del encoder del lanzador:  ", launcher_encoder.getDistance()); */
  }
}
