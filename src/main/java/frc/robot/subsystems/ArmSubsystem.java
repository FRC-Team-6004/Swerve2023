// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MechanismConstants;

import com.ctre.phoenix.CustomParamConfiguration;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubsystem extends SubsystemBase {

  private TalonFX telescope;

  private CANSparkMax pivot;
  private CANSparkMax pivotFollow;

  private DoubleSolenoid solenoid1;
  private DoubleSolenoid solenoid2;

  private SparkMaxPIDController pivotPidController;

  // functions

  public ArmSubsystem() {
    telescope = new WPI_TalonFX(MechanismConstants.kTelescopePort);
    pivot = new CANSparkMax(MechanismConstants.kPivotPort, MotorType.kBrushless);
    pivotFollow = new CANSparkMax(MechanismConstants.kPivotFollowPort, MotorType.kBrushless);

    pivotFollow.follow(pivot, true);

    pivot.setIdleMode(IdleMode.kBrake);
    pivotFollow.setIdleMode(IdleMode.kBrake);

    pivotPidController = pivot.getPIDController();
    
    pivotPidController.setP(MechanismConstants.kPPivot);
    pivotPidController.setI(MechanismConstants.kIPivot);
    pivotPidController.setD(MechanismConstants.kDPivot);
    pivotPidController.setOutputRange(MechanismConstants.kMinPivot, MechanismConstants.kMaxPivot);
    
    telescope.configSelectedFeedbackSensor(
      FeedbackDevice.IntegratedSensor,
      1,
      0
      );

    telescope.setSelectedSensorPosition(0);
    telescope.config_kF(MechanismConstants.kPIDLoopIdxTelescope, MechanismConstants.kFTelescope, MechanismConstants.kTimeoutMsTelescope);
    telescope.config_kP(MechanismConstants.kPIDLoopIdxTelescope, MechanismConstants.kPTelescope, MechanismConstants.kTimeoutMsTelescope);
    telescope.config_kI(MechanismConstants.kPIDLoopIdxTelescope, MechanismConstants.kITelescope, MechanismConstants.kTimeoutMsTelescope);
    telescope.config_kD(MechanismConstants.kPIDLoopIdxTelescope, MechanismConstants.kDTelescope, MechanismConstants.kTimeoutMsTelescope);

    solenoid1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    solenoid2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);

  }
  
  public void setPivotPosition(double angle) {
    pivotPidController.setReference(-1*(angle/360)*MechanismConstants.kReductionPivot, CANSparkMax.ControlType.kPosition);
  }

  public void setTelescopePosition(double percent) { //percent should be 0 to 1
      telescope.set(
        ControlMode.Position, 
        (percent*(MechanismConstants.kRotationsToFullExtentTelescope*MechanismConstants.kReductionTelescope))*MechanismConstants.kTelescopeEncoderCPR
        );
      
  }

  public void toggleGrab(){
    solenoid1.toggle();
    solenoid2.toggle();
  }

  public void setGrab(boolean grab) {
    if(grab){
      solenoid1.set(Value.kReverse);
      solenoid2.set(Value.kForward);
    }
    else {
      solenoid1.set(Value.kForward);
      solenoid2.set(Value.kReverse);
    }
  }

  public void manuelPivot(double speed) {
    pivot.set(speed);
    SmartDashboard.putNumber("Pivot Speed", speed);
  }

  public void manuelTelescope(double speed) {
    telescope.set(ControlMode.PercentOutput, speed);
    SmartDashboard.putNumber("Telescope Speed", speed);

  }

  
  public void pivotOff() {
    pivot.set(0);
    SmartDashboard.putNumber("Pivot Speed", 0);
  }

  public void telescopeOff() {
    telescope.set(ControlMode.PercentOutput, 0);
    SmartDashboard.putNumber("Telescope Speed", 0);

  }

  @Override
  public void periodic() {
      SmartDashboard.putNumber("Telescope Pos", telescope.getSelectedSensorPosition());
      SmartDashboard.putNumber("Pivot Pos", -(pivot.getEncoder().getPosition()/360)*MechanismConstants.kReductionPivot);

  }

}