// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.*;

public class Drivetrain extends SubsystemBase {

  static double motorLimiter = 0.5;
  static WPI_TalonSRX frontLeftMotor = new WPI_TalonSRX(Constants.FRONT_LEFT_MOTOR);
  static WPI_TalonSRX backLeftMotor = new WPI_TalonSRX(Constants.BACK_LEFT_MOTOR);
  static WPI_TalonSRX frontRightMotor = new WPI_TalonSRX(Constants.FRONT_RIGHT_MOTOR);
  static WPI_TalonSRX backRightMotor = new WPI_TalonSRX(Constants.BACK_RIGHT_MOTOR);
  public static MecanumDrive mecanumDrive;
  public static DifferentialDrive differentialDrive;
  private static double targetAngle;
  private static double currentRotationRate;

  static AHRS gyroscope = new AHRS(SPI.Port.kMXP);
  static PIDController angleController;
  // static AnalogGyro gyroscope = new AnalogGyro(Constant  s.GYROSCOPE);

  public Drivetrain() {
    // calls da motors and gives dem da speed controllers but wit da different name
    frontLeftMotor = new WPI_TalonSRX(Constants.FRONT_LEFT_MOTOR);
    backLeftMotor = new WPI_TalonSRX(Constants.BACK_LEFT_MOTOR);
    frontRightMotor = new WPI_TalonSRX(Constants.FRONT_RIGHT_MOTOR);
    backRightMotor = new WPI_TalonSRX(Constants.BACK_RIGHT_MOTOR);

    MotorControllerGroup rightMotors = new MotorControllerGroup(frontRightMotor, backRightMotor);
    MotorControllerGroup leftMotors = new MotorControllerGroup(frontLeftMotor, backLeftMotor);
    differentialDrive = new DifferentialDrive(leftMotors, rightMotors);  
    final double kP = .03;
    final double kI = 0;
    final double kD = 0;
    
    angleController = new PIDController(kP, kI, kD);
    angleController.setTolerance(2);

    // mecanumDrive = new MecanumDrive(frontLeftMotor, backLeftMotor,
    // frontRightMotor, backRightMotor);
  }

  //Angles are measured clockwise from the positive X axis. The robot's speed is independent from its angle or rotation rate.
  //Gyro is feild oreintation while zRotation is relative to the robot
  public static void polDrive(double ySpeed, double xSpeed, double rotationX, double rotationY)
  {
    //calculates polar angle we need to rotate
    targetAngle = Math.toDegrees(Math.atan2(rotationY, rotationX) + Math.PI);
    //converts that angle to a 1 to -1 value
    double rotPower = (targetAngle - 180) / 180;

    //drives the freakin thing
    mecanumDrive.driveCartesian(ySpeed, xSpeed, rotPower, gyroscope.getAngle());
  }

  /**STD()
   * variation of tankdrive that uses a home-made PID system, still in development
   * @param forwardSpeed
   * @param rotationX
   * @param rotationY
   */
  public static void supremeTankDrive(double forwardSpeed, double rotationX, double rotationY)
  {
    // In case of a switch back to analog
    //double calculatedGyroAngle = (gyroscope.getAngle() % 360);
    //if (calculatedGyroAngle > 180){calculatedGyroAngle -= 360;}
    double calculatedGyroAngle = gyroscope.getYaw();

    targetAngle = Math.toDegrees(Math.atan2(rotationX, rotationY) + Math.PI) - calculatedGyroAngle;

    double turnLimiter;
    if (forwardSpeed == 0) {
      turnLimiter = motorLimiter;
    } else {
      turnLimiter = 1 - motorLimiter;
    }
    turnLimiter = 1 - motorLimiter;
    double rotPow = (targetAngle / 180) * turnLimiter;

    frontLeftMotor.set(-forwardSpeed * 0 + rotPow * 0.7);
    backLeftMotor.set(-forwardSpeed* 0 + rotPow * 0.7);
    frontRightMotor.set(forwardSpeed* 0 + rotPow * 0.7);
    backRightMotor.set(forwardSpeed* 0 + rotPow * 0.7);
  }
  /**STDP2BoTW()
   * copy of STD
   * 
   * @param forwardSpeed
   * @param speedLimit
   * @param rotationX
   * @param rotationY
   */
  public static void supremeTankDrivePart2BattleOfTheWheels(double forwardSpeed, double speedLimit, double rotationX,
      double rotationY) {
    double calculatedGyroAngle = (gyroscope.getAngle() % 360);
    if (calculatedGyroAngle > 180) {
      calculatedGyroAngle -= 360;
    }

    targetAngle = Math.toDegrees(Math.atan2(rotationY, rotationX) + Math.PI) - calculatedGyroAngle;

    double turnLimiter;
    if (forwardSpeed == 0) {
      turnLimiter = speedLimit;
    } else {
      turnLimiter = 1 - speedLimit;
    }
    double rotPow = (targetAngle / 180) * turnLimiter;

    frontLeftMotor.set(forwardSpeed - rotPow);
    backLeftMotor.set(forwardSpeed - rotPow);
    frontRightMotor.set(forwardSpeed + rotPow);
    backRightMotor.set(forwardSpeed + rotPow);
  }

  /**classicDrive()
   * simple tankdrive method
   * @param leftY Left Y axis
   * @param rightY right Y axis
   */
  public static void classicDrive(double leftY, double rightY)
  {
    differentialDrive.tankDrive(leftY, -rightY);
  }

  /**chadDrive()
   * adds a function to either the normal "tankdrive" mode or the "arcade" mode depending
   * on driver's preference
   * @param leftY Y axis of the left thumbstick/joystick
   * @param rightY Y axis of the right thumbstick/joystick
   * @param rightX X axis of the right thumbstick/joystick
   * @param button button that rotates the robot to a designated angle
   */
  public static void chadDrive(double leftY,double rightY,double rightX,boolean button)
  {

    if(button)
    {
      targetAngle = Math.toDegrees(Math.atan2(rightY, rightX) + Math.PI);
      System.out.println(targetAngle);
      ultraMegaTurningMethod(targetAngle);
    }
    else
    {
      differentialDrive.tankDrive(-leftY, rightY);
      differentialDrive.arcadeDrive(leftY, rightX);
      currentRotationRate = 0;
    }
  }

  /**zeroGyro()
   * sets a button 
   * @param button button that is used
   */
  public static void zeroGyro(boolean button)
  {
    gyroscope.zeroYaw();
  }

  /**calibrate()
   * 
   * @param button button that is used
   */
  public static void calibrate(boolean button)
  {
    gyroscope.calibrate();
  }

  /** UMTM
   * responsible for rotating the robot to a designated angle 
   * @param angle sets the setpoint for the PID system
   */
  public static void ultraMegaTurningMethod(double angle) {
    System.out.println("UMTM has been called");
    
      if(gyroscope.getYaw() > Math.floor(angle) && gyroscope.getYaw() < Math.ceil(angle))
      {
        currentRotationRate = 0;
        return;
      }

      angleController.setSetpoint((float) angle);
      System.out.println(angle);
      System.out.println(gyroscope.getYaw());
      currentRotationRate = MathUtil.clamp(angleController.calculate(gyroscope.getYaw()), -1, 1);
      differentialDrive.tankDrive(currentRotationRate, currentRotationRate);
    
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
