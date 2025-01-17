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
import frc.robot.Robot;
import frc.robot.utils.RobotMap;
import frc.robot.utils.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
  private static Drivetrain instance;

  private final SwerveModule[] swerveModules;
  private final SwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;

  private SwerveModuleState[] swerveModuleStates;
  private SwerveModulePosition[] swerveModulePositions;

  private final Pigeon2 gyro;
  private double heading;

  public Drivetrain() {
    frontLeftModule = new SwerveModule(RobotMap.CANIVORE_NAME, RobotMap.FRONT_LEFT_MODULE_DRIVE_ID,
        RobotMap.FRONT_LEFT_MODULE_TURN_ID, RobotMap.FRONT_LEFT_MODULE_CANCODER_ID,
        DriveConstants.kFrontLeftCancoderOffset);
    frontRightModule = new SwerveModule(RobotMap.CANIVORE_NAME, RobotMap.FRONT_RIGHT_MODULE_DRIVE_ID,
        RobotMap.FRONT_RIGHT_MODULE_TURN_ID, RobotMap.FRONT_RIGHT_MODULE_CANCODER_ID,
        DriveConstants.kFrontRightCancoderOffset);
    backLeftModule = new SwerveModule(RobotMap.CANIVORE_NAME, RobotMap.BACK_LEFT_MODULE_DRIVE_ID,
        RobotMap.BACK_LEFT_MODULE_TURN_ID, RobotMap.BACK_LEFT_MODULE_CANCODER_ID,
        DriveConstants.kBackLeftCancoderOffset);
    backRightModule = new SwerveModule(RobotMap.CANIVORE_NAME, RobotMap.BACK_RIGHT_MODULE_DRIVE_ID,
        RobotMap.BACK_RIGHT_MODULE_TURN_ID, RobotMap.BACK_RIGHT_MODULE_CANCODER_ID,
        DriveConstants.kBackRightCancoderOffset);

    swerveModules = new SwerveModule[] { frontLeftModule, frontRightModule, backLeftModule, backRightModule };
    swerveModulePositions = new SwerveModulePosition[] { frontLeftModule.getPosition(), frontRightModule.getPosition(),
        backLeftModule.getPosition(), backRightModule.getPosition() };
    swerveModuleStates = DriveConstants.kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));

    gyro = new Pigeon2(RobotMap.GYRO_ID, RobotMap.CANIVORE_NAME);
    gyro.setYaw(0);
  }

  public static Drivetrain getInstance() {
    if (instance == null) {
      instance = new Drivetrain();
    }
    return instance;
  }

  public void updateModulePositions() {
    for (int i = 0; i < swerveModulePositions.length; i++) {
      swerveModulePositions[i] = swerveModules[i].getPosition();
    }
  }

  public void setSwerveModuleStates(SwerveModuleState[] desiredModuleStates) {
    for (int i = 0; i < desiredModuleStates.length; i++) {
      swerveModules[i].setDesiredState(desiredModuleStates[i]);
    }
  }

  public double getHeading() {
    heading = -gyro.getYaw().getValueAsDouble();
    return Math.IEEEremainder(heading, 360);
  }

  public void resetGyro(){
    gyro.reset();
  }

  public Rotation2d getHeadingAsRotation2d() {
    return gyro.getRotation2d();
  }

  public void drive(Translation2d translation, double rotation, boolean fieldOriented, Translation2d centerOfRotation) {
    ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

    ChassisSpeeds robotRelativeSpeeds;
    if (fieldOriented) {
      robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getHeadingAsRotation2d());
    } else {
      robotRelativeSpeeds = fieldRelativeSpeeds;
    }

    swerveModuleStates = DriveConstants.kinematics.toSwerveModuleStates(robotRelativeSpeeds);
    // TODO: desaturate later!
    optimizeModuleStates();
    setSwerveModuleStates(swerveModuleStates);
  }

  public void optimizeModuleStates() {
    for (int i = 0; i < swerveModuleStates.length; i++) {
      swerveModuleStates[i].optimize(new Rotation2d(swerveModules[i].getCANCoderReading()));
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
