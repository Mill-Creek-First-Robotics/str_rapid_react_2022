// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.Constants;
//import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.AnalogGyro;
//import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class Drivetrain extends SubsystemBase {

  static double motorLimiter = 0.5;
  static WPI_TalonSRX frontLeftMotor = null;
  static WPI_TalonSRX backLeftMotor = null;
  static WPI_TalonSRX frontRightMotor = null;
  static WPI_TalonSRX backRightMotor = null;
  public static MecanumDrive mecanumDrive;
  public static DifferentialDrive differentialDrive;
  private static double targetAngle;
  //static AHRS gyroscope = new AHRS(SPI.Port.kMXP);
  static AnalogGyro gyroscope = new AnalogGyro(Constants.GYROSCOPE);


  public Drivetrain() 
  {
    //calls da motors and gives dem da speed controllers but wit da different name
    frontLeftMotor = new WPI_TalonSRX(Constants.FRONT_LEFT_MOTOR);
    backLeftMotor = new WPI_TalonSRX(Constants.BACK_LEFT_MOTOR);
    frontRightMotor = new WPI_TalonSRX(Constants.FRONT_RIGHT_MOTOR);
    backRightMotor = new WPI_TalonSRX(Constants.BACK_RIGHT_MOTOR);

    SpeedControllerGroup rightMotors = new SpeedControllerGroup(frontRightMotor, backRightMotor);
    SpeedControllerGroup leftMotors = new SpeedControllerGroup(frontLeftMotor, backLeftMotor);
    differentialDrive = new DifferentialDrive(leftMotors, rightMotors);

    //mecanumDrive = new MecanumDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
  }

  //Angles are measured clockwise from the positive X axis. The robot's speed is independent from its angle or rotation rate.
  //Gyro is feild oreintation while zRotation is relative to the robot
  /*public static void polDrive(double ySpeed, double xSpeed, double rotationX, double rotationY)
  {
    //calculates polar angle we need to rotate
    angle = Math.toDegrees(Math.atan2(rotationY, rotationX) + Math.PI);
    //converts that angle to a 1 to -1 value
    double rotPower = (angle - 180) / 180;

    //drives the freakin thing
    mecanumDrive.driveCartesian(ySpeed, xSpeed, rotPower, gyroscope.getAngle());
  }*/

  public static void supremeTankDrive(double forwardSpeed, double rotationX, double rotationY)
  {
    double calculatedGyroAngle = (gyroscope.getAngle() % 360);
    if (calculatedGyroAngle > 180){calculatedGyroAngle -= 360;}

    targetAngle = Math.toDegrees(Math.atan2(rotationY, rotationX) + Math.PI) - calculatedGyroAngle;

    double turnLimiter;
    if(forwardSpeed == 0){turnLimiter = motorLimiter;}
    else{turnLimiter = 1 - motorLimiter;}
    double rotPow = (targetAngle / 180) * turnLimiter;

    frontLeftMotor.set(forwardSpeed - rotPow);
    backLeftMotor.set(forwardSpeed - rotPow);
    frontRightMotor.set(forwardSpeed + rotPow);
    backRightMotor.set(forwardSpeed + rotPow);
  }

  public static void supremeTankDrivePart2BattleOfTheWheels(double forwardSpeed, double speedLimit,  double rotationX, double rotationY)
  {
    double calculatedGyroAngle = (gyroscope.getAngle() % 360);
    if (calculatedGyroAngle > 180){calculatedGyroAngle -= 360;}

    targetAngle = Math.toDegrees(Math.atan2(rotationY, rotationX) + Math.PI) - calculatedGyroAngle;

    double turnLimiter;
    if(forwardSpeed == 0){turnLimiter = speedLimit;}
    else{turnLimiter = 1 - speedLimit;}
    double rotPow = (targetAngle / 180) * turnLimiter;


    frontLeftMotor.set(forwardSpeed - rotPow);
    backLeftMotor.set(forwardSpeed - rotPow);
    frontRightMotor.set(forwardSpeed + rotPow);
    backRightMotor.set(forwardSpeed + rotPow);
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
