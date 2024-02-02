// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.IntakeLauncherConstants;

public class IntakeLauncherSubsystem extends SubsystemBase {
  private CANSparkMax m_intakeMotor = new CANSparkMax(IntakeLauncherConstants.intakelauncher_intakeMotor_PORT, MotorType.kBrushless);
  private CANSparkMax m_downMotor = new CANSparkMax(IntakeLauncherConstants.intakelauncher_downMotor_PORT, MotorType.kBrushless);
  private CANSparkMax m_upMotor = new CANSparkMax(IntakeLauncherConstants.intakelauncher_upMotor_PORT, MotorType.kBrushless);

  /** Creates a new IntakeLauncher. */
  public IntakeLauncherSubsystem() {

  }

  public Command setEverythingOFF(){
    return runOnce(()-> {m_intakeMotor.set(0);
                        m_downMotor.set(0);
                        m_upMotor.set(0);});
   
  }

  public Command getPiece(){
    return run(()-> {m_intakeMotor.set(0.8);
                    m_downMotor.set(0);
                    m_upMotor.set(0);});
   
  }
  
  public Command throwPiece(){
    return run(()-> {m_intakeMotor.set(0.8);
                    m_downMotor.set(-1);
                    m_upMotor.set(1);});
   
  }

  public Command holdLaunch(){
    return run(()-> {m_intakeMotor.set(0);
                    m_downMotor.set(-1);
                    m_upMotor.set(1);});
   
  }

  public Command holdPiece(){
    return run(()-> {m_intakeMotor.set(-0.2);
                    m_downMotor.set(0.2);
                    m_upMotor.set(-0.2);});
   
  }

  public Command throwPiece1(){
    return run(()-> m_downMotor.set(-1));
   
  }

  public Command throwPiece2(){
    return run(()-> m_upMotor.set(1));
   
  }

  public Command throwPieceOFF1(){
    return run(()-> m_downMotor.set(0));
   
  }

  public Command throwPieceOFF2(){
    return run(()-> m_upMotor.set(0));
   
  }


  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
