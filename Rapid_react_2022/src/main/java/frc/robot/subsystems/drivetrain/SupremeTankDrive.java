// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.subsystems.drivetrain.Drivetrain;
public class SupremeTankDrive extends SubsystemBase {

  private static double targetAngle;
  private static AHRS gyroscope;
  private static double motorLimiter = 0.5; 
  public SupremeTankDrive() 
  {
    gyroscope = new AHRS(SPI.Port.kMXP);
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

    Drivetrain.frontLeftMotor.set(-forwardSpeed * 0 + rotPow * 0.7);
    Drivetrain.backLeftMotor.set(-forwardSpeed* 0 + rotPow * 0.7);
    Drivetrain.frontRightMotor.set(forwardSpeed* 0 + rotPow * 0.7);
    Drivetrain.backRightMotor.set(forwardSpeed* 0 + rotPow * 0.7);
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

    Drivetrain.frontLeftMotor.set(forwardSpeed - rotPow);
    Drivetrain.backLeftMotor.set(forwardSpeed - rotPow);
    Drivetrain.frontRightMotor.set(forwardSpeed + rotPow);
    Drivetrain.backRightMotor.set(forwardSpeed + rotPow);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}