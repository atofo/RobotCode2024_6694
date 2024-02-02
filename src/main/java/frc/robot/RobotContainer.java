// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Commands.DriveWithJoystick;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.IntakeLauncherSubsystem;

public class RobotContainer {

  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.controllerPort);

  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final DriveWithJoystick m_DriveWithJoystick = new DriveWithJoystick(m_drivetrainSubsystem,
      () -> m_driverController.getRawAxis(1), () -> m_driverController.getRawAxis(0),
      () -> m_driverController.getRawAxis(4)); 

  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
  private final IntakeLauncherSubsystem m_IntakeLauncherSubsystem = new IntakeLauncherSubsystem();
  private Trigger xButton = m_driverController.x();
  private Trigger bButton = m_driverController.b();
  private Trigger aButton = m_driverController.a();
  private Trigger yButton = m_driverController.y();

  private Trigger povRight = m_driverController.povRight();
  private Trigger povLeft = m_driverController.povLeft();
  private Trigger povDown = m_driverController.povDown();
  private Trigger povUp = m_driverController.povUp();
  private Trigger L3 = m_driverController.leftStick();

  private boolean ON = false;

  public RobotContainer() {
    m_drivetrainSubsystem.setDefaultCommand(m_DriveWithJoystick);
    xButton.whileTrue(m_ArmSubsystem.setSetpoint(62)); //Setpoint = 82 para llegar hacia abajo
    bButton.whileTrue(m_ArmSubsystem.setSetpoint(82.5));
    aButton.whileTrue(m_IntakeLauncherSubsystem.getPiece());
    yButton.whileTrue(m_IntakeLauncherSubsystem.throwPiece1());
    povUp.whileTrue(m_IntakeLauncherSubsystem.throwPiece2());
    L3.whileTrue(m_ArmSubsystem.setSetpoint(0));
    povRight.whileTrue(m_IntakeLauncherSubsystem.throwPieceOFF1());
    povLeft.whileTrue(m_IntakeLauncherSubsystem.throwPieceOFF2());
    povDown.whileTrue(m_IntakeLauncherSubsystem.getPieceOFF());

   


    /* povRight.onTrue(m_ArmSubsystem.setSetpointManual(povLeft, povRight));
    povLeft.onTrue(m_ArmSubsystem.setSetpointManual(povLeft, povRight)); */
 
    configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
