// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.drivetrain.*;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.drivetrain.Drivetrain;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.commands.*;
import frc.robot.commands.intake.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final ChadDrive m_chadDrive = new ChadDrive();
  private final Intake m_intake = new Intake();
  private final Command m_autoCommand = null;
  final XboxController stick = new XboxController(Constants.CONTROLLER);
  
  JoystickButton aButton = new JoystickButton(stick, 1);
  JoystickButton bButton = new JoystickButton(stick, 2);
  JoystickButton xButton = new JoystickButton(stick, 3);
  JoystickButton yButton = new JoystickButton(stick, 4);
  JoystickButton leftBumper = new JoystickButton(stick, 5);
  JoystickButton rightBumper = new JoystickButton(stick, 6);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() 
  {
    
    rightBumper.whenPressed(new toggleSlowMode(m_drivetrain));
    bButton.whenPressed(new ToggleMotor(m_intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
