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
import frc.robot.Commands.IntakeLauncherCommands;
import frc.robot.Commands.Intake_getNote;
import frc.robot.Commands.Intake_returnNote;
import frc.robot.Commands.LauncherWithJoystick;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.IntakeLauncherSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.LauncherSubsystem;

public class RobotContainer {

  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.controllerPort);

  private Trigger xButton = m_driverController.x();
  private Trigger bButton = m_driverController.b();
  private Trigger aButton = m_driverController.a();
  private Trigger yButton = m_driverController.y();

  private Trigger LB = m_driverController.leftBumper();
  private Trigger RB = m_driverController.rightBumper();

  private Trigger povRight = m_driverController.povRight();
  private Trigger povLeft = m_driverController.povLeft();
  private Trigger povDown = m_driverController.povDown();
  private Trigger povUp = m_driverController.povUp();

  private Trigger L3 = m_driverController.leftStick();

  
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final DriveWithJoystick m_DriveWithJoystick = new DriveWithJoystick(m_drivetrainSubsystem,
      () -> m_driverController.getRawAxis(1), () -> m_driverController.getRawAxis(0),
      () -> m_driverController.getRawAxis(4), () -> m_driverController.getRawAxis(3),
      () -> m_driverController.getRawAxis(2));

  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();

  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final Intake_getNote m_Intake_getNote = new Intake_getNote(m_IntakeSubsystem);
  private final Intake_returnNote m_Intake_returnNote = new Intake_returnNote(m_IntakeSubsystem);

  private final LauncherSubsystem m_LauncherSubsystem = new LauncherSubsystem();
  private final LauncherWithJoystick m_LauncherWithJoystick = new LauncherWithJoystick(m_LauncherSubsystem);

  public RobotContainer() {

    m_drivetrainSubsystem.setDefaultCommand(m_DriveWithJoystick);
   
    povUp.whileTrue(m_ArmSubsystem.setSetpoint(0.37)); // Parado
    povDown.whileTrue(m_ArmSubsystem.setSetpoint(0.21)); // Shoot
    L3.whileTrue(m_ArmSubsystem.setSetpoint(0.09)); // Intake/Modo Correr

    bButton.whileTrue(m_Intake_getNote);
    aButton.whileTrue(m_Intake_returnNote);

    RB.onTrue(m_LauncherWithJoystick);
    
    /* povRight.onTrue(m_ArmSubsystem.setSetpointManual(povLeft, povRight));
     * povLeft.onTrue(m_ArmSubsystem.setSetpointManual(povLeft, povRight));
     */

    configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
