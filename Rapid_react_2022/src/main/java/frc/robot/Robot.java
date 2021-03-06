// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.intake.touchingLimit;
import frc.robot.subsystems.drivetrain.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.intake.Intake;
/*
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private Intake m_intake = new Intake();
  private ChadDrive m_chadDrive = new ChadDrive();
  private Drivetrain m_drivetrain = new Drivetrain();
  private XboxController stick = new XboxController(Constants.CONTROLLER);
  private double startTime;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();


  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    Drivetrain.classicDrive(0, 0);
    startTime = Timer.getFPGATimestamp();
    Intake.enabled(false);
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if(Timer.getFPGATimestamp() - startTime < 5.0) {
      Drivetrain.tuxDrive(0, -.7);
    } else {
      Drivetrain.classicDrive(0, 0);
    }
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    Intake.enabled(false);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //m_drivetrain.classicDrive(-stick.getLeftY() * .9, -stick.getRightY() * .9);
    m_drivetrain.tuxDrive(stick.getRightX(), stick.getLeftY() * -1);
    //if(stick.getRawButton(5)){m_drivetrain.toggleSlow();}
    m_drivetrain.holdTurbo(stick.getRawButton(6));
    m_drivetrain.holdTurbo(stick.getRawButton(5));
    //m_chadDrive.zeroGyro(stick.getRawButton(2));
    //m_chadDrive.calibrate(stick.getRawButton(3));
    if(stick.getRawButton(1)){Intake.switchArm();}
    //m_drivetrain.supremeTankDrive(stick.getLeftY(), stick.getRightX(), stick.getRightY());
    new touchingLimit(m_intake);     
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() 
  {
    
  }


}
