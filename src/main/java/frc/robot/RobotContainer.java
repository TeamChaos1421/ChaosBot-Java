// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.AmpDump;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  // Subsystems
  private final Intake m_intake = new Intake();
  private final Climber m_climber = new Climber();
  private final AmpDump m_ampDump = new AmpDump();
  private final Shooter m_shooter = new Shooter();
  private final Drivetrain m_Drivetrain = new Drivetrain();

  // Xbox Controllers
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_codriverController = 
      new CommandXboxController(OperatorConstants.kCodriverControllerPort);



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_Drivetrain.setDefaultCommand(
      m_Drivetrain.Drive(
        m_driverController.getLeftX(),
        m_driverController.getLeftY(),
        m_driverController.getRightX(),
        !m_driverController.leftBumper().getAsBoolean())
    );
    // Configure the trigger bindings
    configureBindings();
  }

  // Control Bindings
  private void configureBindings() {

    // Zero Gyro
    m_driverController.y().debounce(0.1)
      .onTrue(m_Drivetrain.ZeroHeading());

    // Intake on
    m_codriverController.a()
      .or(m_codriverController.b())
      .or(m_codriverController.y())
        .whileTrue(m_intake.On());

    // Load for Amp
    m_codriverController.b()
      .whileTrue(m_shooter.Load());

    // Shoot for Speaker
    m_codriverController.y()
      .whileTrue(m_shooter.Shoot());

    // Dump to Amp
    m_codriverController.x().debounce(0.1)
      .toggleOnTrue(m_ampDump.Toggle());

    // Co-driver zeroing controls
    m_codriverController.b()
      .and(m_codriverController.y())
        .whileFalse(m_shooter.Zero());
    
    // Climber controls
    m_codriverController.rightTrigger(0.1)
      .or(m_codriverController.leftTrigger(0.1))
        .onTrue(m_climber.Set(
          m_codriverController.getRightTriggerAxis() - m_codriverController.getLeftTriggerAxis()
        ));
  }

  // Get Auto
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto();
  }
}
