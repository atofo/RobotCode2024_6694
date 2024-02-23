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
import frc.robot.Commands.Intake_getNote;
import frc.robot.Commands.Intake_returnNote;
import frc.robot.Commands.LauncherWithJoystick;
import frc.robot.Commands.LeftClimberDown;
import frc.robot.Commands.LeftClimberUp;
import frc.robot.Commands.RightClimberUp;
import frc.robot.Commands.RightClimberDown;
import frc.robot.Commands.Intake_ThrowNote;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.LauncherSubsystem;
import frc.robot.Subsystems.LeftClimber;
import frc.robot.Subsystems.RightClimber;
import frc.robot.Subsystems.ShooterSubsystem;

public class RobotContainer {

  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.controllerPort);

  private Trigger start = m_driverController.start();
  private Trigger back = m_driverController.back();

  private Trigger xButton = m_driverController.x();
  private Trigger bButton = m_driverController.b();
  private Trigger aButton = m_driverController.a();
  private Trigger yButton = m_driverController.y();

  private Trigger LB = m_driverController.leftBumper();
  private Trigger RB = m_driverController.rightBumper();
  private Trigger RT = m_driverController.rightTrigger();
  private Trigger LT = m_driverController.leftTrigger();


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

/*   private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem(); */

  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final Intake_getNote m_Intake_getNote = new Intake_getNote(m_IntakeSubsystem);
  private final Intake_returnNote m_Intake_returnNote = new Intake_returnNote(m_IntakeSubsystem);
  private final Intake_ThrowNote m_Intake_throwNote = new Intake_ThrowNote(m_IntakeSubsystem);
  


 /*  private final LeftClimber m_LeftClimberSubsystem = new LeftClimber();
  private final RightClimber m_RightClimberSubsystem = new RightClimber();

  private final LeftClimberDown m_LeftClimberDown = new LeftClimberDown(m_LeftClimberSubsystem);
  private final LeftClimberUp m_LeftClimberUp = new LeftClimberUp(m_LeftClimberSubsystem);
  private final RightClimberDown m_RightClimberDown = new RightClimberDown(m_RightClimberSubsystem);
  private final RightClimberUp m_RightClimberUp = new RightClimberUp(m_RightClimberSubsystem); */

private final ShooterSubsystem m_shooter = new ShooterSubsystem();

  private final Command m_spinUpShooter = Commands.runOnce(m_shooter::enable, m_shooter);
  private final Command m_stopShooter = Commands.runOnce(m_shooter::disable, m_shooter);

  public RobotContainer() {

    m_drivetrainSubsystem.setDefaultCommand(m_DriveWithJoystick);
   
   /*  povUp.whileTrue(m_ArmSubsystem.setSetpoint(0.60)); // 90 degrees
    povRight.whileTrue(m_ArmSubsystem.setSetpoint(0.425)); // 90 degrees
    povDown.whileTrue(m_ArmSubsystem.setSetpoint(0.279)); // Shoot
    L3.whileTrue(m_ArmSubsystem.setSetpoint(0.13)); // Intake/Modo Correr */
    //RT.onTrue(m_Arm_manualSetpoint);

    yButton.whileTrue(m_Intake_throwNote);
    bButton.toggleOnTrue(m_Intake_getNote); //Intake get Note
    aButton.whileTrue(m_Intake_returnNote); //Intake return Note  
/*     yButton.whileTrue(m_LauncherWithJoystick); //Intake return Note  
 */
    //RB.toggleOnTrue(m_LauncherWithJoystick); //Toggle Shoot

    /* start.whileTrue(m_RightClimberUp); 
    back.whileTrue(m_LeftClimberUp);
    xButton.whileTrue(m_RightClimberDown);
    LB.whileTrue(m_LeftClimberDown); */

    /* aButton.whileTrue(m_LauncherSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    bButton.whileTrue(m_LauncherSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    xButton.whileTrue(m_LauncherSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    yButton.whileTrue(m_LauncherSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse)); */

    configureBindings();
  }

  private void configureBindings() {
   RB.onTrue(m_spinUpShooter); //Empezar a girar lanzador
    LB.onTrue(m_stopShooter); //Parar lanzador


    Command shoot =
      Commands.either(
            // Run the feeder
            Commands.runOnce(m_IntakeSubsystem::throwNote, m_IntakeSubsystem), //Va a lanzar si se alcanza la velocidad deseada
            // Do nothing
            Commands.none(),
            // Determine which of the above to do based on whether the shooter has reached the
            // desired speed
            m_shooter::atSetpoint);

    Command stopIntake = Commands.runOnce(m_IntakeSubsystem::intakeOFF, m_IntakeSubsystem);

    //disparar cuando se presione el boton X
    xButton.onTrue(shoot).onFalse(stopIntake);


  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
