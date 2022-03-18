// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import javax.lang.model.util.ElementScanner6;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  static // calls da roller motor and assigns it a speed controller
  double el_speed = -0.55;
  static WPI_TalonSRX roller = new WPI_TalonSRX(Constants.ROLLER_MOTOR);
  static WPI_TalonSRX arm = new WPI_TalonSRX(Constants.ARM_MOTOR);
  static boolean toggled = false;
  static int armPosition = 0;
  static DigitalInput topSwitch = new DigitalInput(0);
  static boolean armOn = true;
  //static DigitalInput bottomSwitch = new DigitalInput(1);
  public Intake() {
    roller = new WPI_TalonSRX(Constants.ROLLER_MOTOR);
    toggled = false;
    roller.set(0);
  }

  public void setSpeed(double speed) {
    // sets the roller motors speed
    roller.set(speed);
  }

  public void inverseElSpeed() {
    el_speed = el_speed * -1;
    roller.set(el_speed);
  }

  public static void updateToggle() {
    // if toggle is pressed toggled is opposite of its original state
    toggled = !toggled;

    // if toggled is on it will set to default speed
    // else toggled must be off and speed is zero
    if (toggled) {
      roller.set(el_speed);
    } else {
      roller.set(0);
    }
  }

  public static void raiseArm()
  {
    if(!(topSwitch.get()))
      arm.set(1);
      armOn = true;
  }
  public static void lowerArm()
  {
    arm.set(-1);
    armOn = true;
  }

  public static void switchArm()
  {

    if(armPosition == 1)
    {
      armPosition = 0;
      lowerArm();
    }
    else if(armPosition == 0)
    {
      armPosition = 1;
      raiseArm();
    }
    else{
      System.out.println("¯\\_(ツ)_/¯");
    }
  }

  public static void touching() {
    if(topSwitch.get()) {
      arm.set(0);
      armOn = false;
    }
    if(armPosition == 1 && armOn == false) {
      raiseArm();
    }
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
