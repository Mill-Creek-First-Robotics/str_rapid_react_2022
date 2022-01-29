// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.Constants;
import com.kauailabs.navx.frc.*;
import frc.robot.subsystems.drivetrain.PIDgyro;

public class Drivetrain extends SubsystemBase {

  WPI_TalonSRX frontLeftMotor = null;
  WPI_TalonSRX backLeftMotor = null;
  WPI_TalonSRX frontRightMotor = null;
  WPI_TalonSRX backRightMotor = null;
  public static MecanumDrive mecanumDrive;
  private static double angle = 0;
  static AHRS gyroscope = new AHRS(SPI.Port.kMXP);


  public Drivetrain() 
  {
    //calls da motors and gives dem da speed controllers but wit da different name
    frontLeftMotor = new WPI_TalonSRX(Constants.FRONT_LEFT_MOTOR);
    backLeftMotor = new WPI_TalonSRX(Constants.BACK_LEFT_MOTOR);
    frontRightMotor = new WPI_TalonSRX(Constants.FRONT_RIGHT_MOTOR);
    backRightMotor = new WPI_TalonSRX(Constants.BACK_RIGHT_MOTOR);
    mecanumDrive = new MecanumDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
  }

  //Angles are measured clockwise from the positive X axis. The robot's speed is independent from its angle or rotation rate.
  //Gyro is feild oreintation while zRotation is relative to the robot
  public static void polDrive(double ySpeed, double xSpeed, double rotationX, double rotationY)
  {
    //calculates polar angle we need to rotate
    //wait a second... this is pointless
    angle = Math.toDegrees(Math.atan2(rotationY, rotationX) + Math.PI);
    double rotPower = (angle - 180) / 180;



    //drives the freakin thing
    mecanumDrive.driveCartesian(ySpeed, xSpeed, rotPower, gyroscope.getAngle());
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
