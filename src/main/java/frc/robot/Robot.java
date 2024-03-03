// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.urcl.URCL;

import edu.wpi.first.wpilibj.DataLogManager;



import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;



  @Override
  public void robotInit() {


     
  /*DataLogManager.start();+
    URCL.start(); */
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    if(m_robotContainer.m_shooter.atSetpoint() && m_robotContainer.m_ArmSubsystem.atSetpoint()){
      m_robotContainer.port2.set(false);
      m_robotContainer.port1.set(true);
      m_robotContainer.port0.set(true);
    }
    else if(m_robotContainer.m_shooter.charging() == true){
      m_robotContainer.port2.set(false);
      m_robotContainer.port1.set(true);
      m_robotContainer.port0.set(false);
    }
    else if(m_robotContainer.m_drivetrainSubsystem.inverted() == true){
      m_robotContainer.port2.set(true);
      m_robotContainer.port1.set(false);
      m_robotContainer.port0.set(false);
    }
    else if (m_robotContainer.m_IntakeSubsystem.noteIn() == true) {
      m_robotContainer.port2.set(false);
      m_robotContainer.port1.set(false);
      m_robotContainer.port0.set(true);
    } else {
      m_robotContainer.port2.set(false);
      m_robotContainer.port1.set(false);
      m_robotContainer.port0.set(false);
    }

    SmartDashboard.putBoolean("PORT2: ", m_robotContainer.port2.get());
    SmartDashboard.putBoolean("PORT1: ", m_robotContainer.port1.get());
    SmartDashboard.putBoolean("PORT0: ", m_robotContainer.port0.get());


  }
  
  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
