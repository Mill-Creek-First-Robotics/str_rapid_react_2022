// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ChadDrive extends SubsystemBase {
  private static double currentRotationRate;
  //static AHRS gyroscope = new AHRS(SPI.Port.kMXP);
  static PIDController angleController;
  
  /** Creates a new ExampleSubsystem. */
  public ChadDrive() 
  {
    
    final double kP = .005;
    final double kI = 0;
    final double kD = 0;
    
    angleController = new PIDController(kP, kI, kD);
    angleController.setTolerance(1);
  }

/**chadDrive()
   * adds a function to either the normal "tankdrive" mode or the "arcade" mode depending
   * on driver's preference
   * @param leftY Y axis of the left thumbstick/joystick
   * @param rightY Y axis of the right thumbstick/joystick
   * @param rightX X axis of the right thumbstick/joystick
   */
  public static void chadDrive(double leftY,double rightY,int POV, double rightX)
  {
	//Drivetrain.differentialDrive.tankDrive(leftY, rightY);
    Drivetrain.differentialDrive.arcadeDrive(rightX, leftY);
    currentRotationRate = 0;
  }

  /** UMTM
   * responsible for rotating the robot to a designated angle 
   * @param angle sets the setpoint for the PID system
   * @deprecated
   */
  /*public static void ultraMegaTurningMethod(double angle)
  {
    System.out.println("UMTM has been called");


      angleController.setSetpoint((float) angle);
      System.out.println(angle);
      System.out.println(gyroscope.getYaw());
      currentRotationRate = angleController.calculate(gyroscope.getYaw());
      Drivetrain.differentialDrive.arcadeDrive(currentRotationRate, 0);
    
  }*/
 
  /**zeroGyro()
   * sets a button 
   * @param button button that is used
   * @deprecated
   */
  /*public static void zeroGyro(boolean button)
  {
    if(button)
      gyroscope.zeroYaw();
  }*/

  /**calibrate()
   * 
   * @param button button that is used
   * @deprecated
   */
  /*public static void calibrate(boolean button)
  {
    if(button)
      gyroscope.calibrate();
  }*/



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
