// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  // Subsystems
  private final Intake m_intake = new Intake();
  private final Climber m_climber = new Climber();
  private final Shooter m_shooter = new Shooter();
  private final Drivetrain m_Drivetrain = new Drivetrain();

  private final SendableChooser<Command> autoChooser;

  // Xbox Controllers
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_codriverController = 
      new CommandXboxController(OperatorConstants.kCodriverControllerPort);



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_shooter.Init();

    m_Drivetrain.setDefaultCommand(
      new TeleopSwerve(
        m_Drivetrain,
        () -> m_driverController.getLeftX(),
        () -> -m_driverController.getLeftY(),
        () -> m_driverController.getRightX(),
        () -> m_driverController.leftBumper().getAsBoolean()
      )
    );
    // Configure the trigger bindings
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
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
        .whileTrue(m_intake.On())
        .onFalse(m_intake.Off());

    // Load for Amp
    m_codriverController.b()
      .whileTrue(
        m_shooter.ResetTimer()
          .andThen(m_shooter.Load()))
        .onFalse(m_shooter.Reset());

    // Shoot for Speaker
    m_codriverController.y()
      .whileTrue(m_shooter.Shoot())
      .onFalse(m_shooter.Zero());
    
    // Climber controls
    m_codriverController.rightTrigger()
      .whileTrue(m_climber.Set(1))
        .onFalse(m_climber.Set(0));
    m_codriverController.leftTrigger()
      .whileTrue(m_climber.Set(-1))
        .onFalse(m_climber.Set(0));
  }

  // Get Auto
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto();
  }
}
