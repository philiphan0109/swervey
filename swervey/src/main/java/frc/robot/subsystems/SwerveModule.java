// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;
import frc.robot.utils.Kraken;

public class SwerveModule extends SubsystemBase {
  
  private final CANcoder canCoder;

  private final int drivingCANId;
  private final int steerCANId;
  private final int CANCoderId;

  private final Kraken driveMotor;
  private final Kraken steerMotor;

  private SwerveModuleState desiredState;
  private double moduleAngularOffset;
  
  public SwerveModule(String canbusName, int drivingCANId, int steerCANId, int CANCoderId, double moduleAngularOffset) {
    this.drivingCANId = drivingCANId;
    this.steerCANId = steerCANId;
    this.CANCoderId = CANCoderId;
    this.moduleAngularOffset = moduleAngularOffset;

    driveMotor = new Kraken(this.drivingCANId, canbusName);
    steerMotor = new Kraken(this.steerCANId, canbusName);

    driveMotor.setInverted(true);
    steerMotor.setInverted(true);

    driveMotor.setSupplyCurrentLimit(ModuleConstants.kDriveMotorCurrentLimit);
    steerMotor.setSupplyCurrentLimit(ModuleConstants.kSteerMotorCurrentLimit);

    driveMotor.setBrake();
    steerMotor.setBrake();

    driveMotor.setEncoder(0);
    steerMotor.setEncoder(0);

    canCoder = new CANcoder(CANCoderId, canbusName);
    configureCANCoder();

    steerMotor.setContinuousOutput();
    steerMotor.setFeedbackDevice(CANCoderId, FeedbackSensorSourceValue.FusedCANcoder);
    
    driveMotor.setVelocityConversionFactor(ModuleConstants.kDriveEncoderVelocityFactor);
    steerMotor.setRotorToSensorRatio(ModuleConstants.kSteerMotorReduction);
    steerMotor.setSensorToMechanismRatio(1.0);
  }

  public void configureCANCoder(){
    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    config.MagnetSensor.MagnetOffset = -moduleAngularOffset/ (2 * Math.PI);
    canCoder.getConfigurator().apply(config);
  }

  public double getCANCoderReading(){
    return  2 * Math.PI * canCoder.getAbsolutePosition().getValueAsDouble();
  }

  public SwerveModuleState getState(){
    return new SwerveModuleState(driveMotor.getMPS(), new Rotation2d(getCANCoderReading()));
  }

  public void setDesiredState(SwerveModuleState desiredModuleState){
    desiredState = desiredModuleState;
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