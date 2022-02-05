// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  //calls da roller motor and assigns it a speed controller
  double el_speed = 0.25;
  WPI_TalonSRX roller = new WPI_TalonSRX(Constants.ROLLER_MOTOR);
  boolean toggled;

  public Intake() 
  {
    roller = new WPI_TalonSRX(Constants.ROLLER_MOTOR);
    toggled = false;
  }


  public void setSpeed(double speed)
  {
    //sets the roller motors speed
    roller.set(speed);
  }

  public void inverseElSpeed() {
    el_speed = el_speed * -1;
    if(toggled) {
      roller.set(el_speed);
    }
  }

  public void updateToggle(boolean isPressed) {
    //if toggle is pressed toggled is opposite of its original state
    if(isPressed) {
      toggled = !toggled;
    }

    // if toggled is on it will set to default speed
    //else toggled must be off and speed is zero
    if(toggled) {
      roller.set(el_speed);
    } else {
      roller.set(0);
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
