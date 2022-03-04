// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;

import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.*;
/** An example command that uses an example subsystem. */
public class Autonomous extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain m_drivetrain;
  private final Intake m_intake;
  boolean yes = true;
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
    Drivetrain.classicDrive(.3, .3);
    wait(500);
    Drivetrain.classicDrive(0, 0);
    Intake.updateToggle();
    Drivetrain.classicDrive(.5, .5);
    wait(1000);
    Drivetrain.classicDrive(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

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
}


private static void wait(int mili)
{
  try
    {
      Thread.sleep(mili);
    }
    catch(InterruptedException ex)
    {
      Thread.currentThread().interrupt();
    }
}