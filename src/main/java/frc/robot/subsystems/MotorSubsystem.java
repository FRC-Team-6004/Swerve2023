// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;

public class MotorSubsystem extends SubsystemBase {
  //Variables
  //int myNumber = 1;
  private TalonFX motor;
  private PIDController pidController;
  private int p = 1;
  private int i = 0;
  private int d = 0;

  //functions

  public MotorSubsystem(int motorId) {
    //initialize stuff
    motor = new TalonFX(motorId);
    pidController = new PIDController(p, i, d);
  }

  public void run(double speed) {
    motor.set(TalonFXControlMode.PercentOutput, speed);
  }
  
  public void stop() {
    motor.set(TalonFXControlMode.PercentOutput, 0);
  }

  public void goTo(double target) {
    motor.set(TalonFXControlMode.PercentOutput, pidController.calculate(motor.getSelectedSensorPosition(),target));
  }
}