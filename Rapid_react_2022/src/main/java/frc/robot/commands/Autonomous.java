// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.Timer;

import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.*;
/** An example command that uses an example subsystem. */
public class Autonomous extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain m_drivetrain;
  private final Intake m_intake;
  private final ScheduleCommand m_scheduler = new ScheduleCommand(new WaitCommand(1));
  private long createdMillis = System.currentTimeMillis();
  boolean yes = true;
  private double startTime;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Autonomous(Drivetrain drivetrain, Intake intake) {
    m_drivetrain = drivetrain;
    m_intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    Drivetrain.classicDrive(0, 0);
    Intake.updateToggle();
    startTime = Timer.getFPGATimestamp();
    //m_scheduler.schedule();
    //Drivetrain.classicDrive(-.5, .5);
    //m_scheduler.schedule();
    //Drivetrain.classicDrive(0, 0);SSS
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(Timer.getFPGATimestamp() - startTime < 1.0) {
      Drivetrain.classicDrive(-0.5, .5);
    } else {
      Drivetrain.classicDrive(0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }

  public void restartTimer()
  {
    createdMillis = System.currentTimeMillis();
  }

  public int getAgeInSeconds() 
  {
      long nowMillis = System.currentTimeMillis();
      return (int)((nowMillis - this.createdMillis) / 1000);
  }
}