// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.motorcontrol.*;

public class Drivetrain extends SubsystemBase {


  static WPI_TalonSRX   frontLeftMotor = new WPI_TalonSRX(Constants.FRONT_LEFT_MOTOR);

  static WPI_TalonSRX backLeftMotor = new WPI_TalonSRX(Constants.BACK_LEFT_MOTOR);
  static WPI_TalonSRX frontRightMotor = new WPI_TalonSRX(Constants.FRONT_RIGHT_MOTOR);
  static WPI_TalonSRX backRightMotor = new WPI_TalonSRX(Constants.BACK_RIGHT_MOTOR);
  public static MecanumDrive mecanumDrive;
  private static boolean slowMode = false;
  private static boolean turboMode = false;
  public static MotorControllerGroup rightMotors = new MotorControllerGroup(frontRightMotor, backRightMotor);
  static MotorControllerGroup leftMotors = new MotorControllerGroup(frontLeftMotor, backLeftMotor);
  public static DifferentialDrive differentialDrive = new DifferentialDrive(leftMotors, rightMotors);
  public Drivetrain() {
    // calls da motors and gives dem da speed controllers but wit da different name
    

  }

  // Angles are measured clockwise from the positive X axis. The robot's speed is
  // independent from its angle or rotation rate.
  // Gyro is feild oreintation while zRotation is relative to the robot

  /**
   * classicDrive() simple tankdrive method
   * 
   * @param leftY  Left Y axis
   * @param rightY right Y axis
   */
  public static void classicDrive(double leftY, double rightY)
  {
    differentialDrive.tankDrive(leftY, -rightY);
  }

  public static void tuxDrive(double leftY, double rightX)
  {
    if(turboMode)
      differentialDrive.arcadeDrive(leftY, rightX);
    else
      differentialDrive.arcadeDrive(leftY * .8, rightX * .8);
  }

  public static void toggleSlow()
  {
    slowMode = !slowMode;
  }

  public static void holdTurbo(boolean button)
  {
    if(button)
    {
      turboMode = true;
    }
    else
    {
      turboMode = false;
    }
  }
  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() 
  {
    // This method will be called once per scheduler run during simulation
  }
}
