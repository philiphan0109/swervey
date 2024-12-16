// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
  private static Drivetrain instance;

  private final SwerveModule[] swerveModules;
  private final SwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;

  private SwerveModuleState[] swerveModuleStates;
  private SwerveModulePosition[] swerveModulePositions;

  private final Pigeon2 gyro;
  private double heading;


  public Drivetrain() {
    frontLeftModule = new SwerveModule("temp", 0, 0, 0, 0);
    frontRightModule = new SwerveModule("temp", 0, 0, 0, 0);
    backLeftModule = new SwerveModule("temp", 0, 0, 0, 0);
    backRightModule = new SwerveModule("temp", 0, 0, 0, 0);

    swerveModules = new SwerveModule[] {frontLeftModule, frontRightModule, backLeftModule, backRightModule};
    swerveModulePositions = new SwerveModulePosition[] {frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition()};
    swerveModuleStates = DriveConstants.kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));

    gyro = new Pigeon2(0);
    gyro.setYaw(0);
  }

  public static Drivetrain getInstance(){
    if(instance == null) {
      instance = new Drivetrain();
    }
    return instance;
  }

  public void updateModulePositions(){
    for(int i = 0; i < swerveModulePositions.length; i++){
      swerveModulePositions[i] = swerveModules[i].getPosition();
    }
  }

  public void setSwerveModuleStates(SwerveModuleState[] desiredModuleStates){
    for(int i = 0; i < desiredModuleStates.length; i++){
      swerveModules[i].setDesiredState(desiredModuleStates[i]);
    }
  }

  public double getHeading(){
    heading = gyro.getAngle();
    return Math.IEEEremainder(heading, 360);
  }

  public Rotation2d getHeadingAsRotation2d(){
    return gyro.getRotation2d();
  }

  public void drive(Translation2d translation, double rotation, boolean fieldOriented, Translation2d centerOfRotation){
    ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

    ChassisSpeeds robotRelativeSpeeds;
    if(fieldOriented){
      robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getHeadingAsRotation2d());
    } else {
      robotRelativeSpeeds = fieldRelativeSpeeds;
    }

    swerveModuleStates = DriveConstants.kinematics.toSwerveModuleStates(robotRelativeSpeeds);
    // TODO: desaturate later!
    optimizeModuleStates();
    setSwerveModuleStates(swerveModuleStates);
  }

  public void optimizeModuleStates(){
    for(int i = 0; i < swerveModuleStates.length; i++){
      swerveModuleStates[i] = SwerveModuleState.optimize(swerveModuleStates[i], 
                                                            new Rotation2d(swerveModules[i].getCANCoderReading()));
    }
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateModulePositions();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
