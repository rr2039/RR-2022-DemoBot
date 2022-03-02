// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.interfaces.Drivetrain;

public class DrivetrainSparkMax extends SubsystemBase implements Drivetrain {

  CANSparkMax leftSpark1 = null;
  CANSparkMax leftSpark2 = null;
  CANSparkMax rightSpark1 = null;
  CANSparkMax rightSpark2 = null;

  RelativeEncoder leftEnc = null;
  RelativeEncoder rightEnc = null;

  SparkMaxPIDController leftPID = null;
  SparkMaxPIDController rightPID = null;

  AHRS drivetrainGyro = null;

  DifferentialDrive differentialDrive = null;

  double openRampRate = 0.2;
  double closedRampRate = 2.0;

  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new DrivetrainSparkMax. */
  public DrivetrainSparkMax() {
    leftSpark1 = new CANSparkMax(Constants.DRIVETRAIN_LEFT_FRONT, MotorType.kBrushless);
    leftSpark2 = new CANSparkMax(Constants.DRIVETRAIN_LEFT_BACK, MotorType.kBrushless);
    rightSpark1 = new CANSparkMax(Constants.DRIVETRAIN_RIGHT_FRONT, MotorType.kBrushless);
    rightSpark2 = new CANSparkMax(Constants.DRIVETRAIN_RIGHT_BACK, MotorType.kBrushless);
    
    //leftSpark1.setClosedLoopRampRate(Preferences.getDouble("Ramp", closedRampRate));
    leftSpark1.setOpenLoopRampRate(Preferences.getDouble("Ramp", openRampRate));
    //leftSpark2.setClosedLoopRampRate(Preferences.getDouble("Ramp", closedRampRate));
    leftSpark2.setOpenLoopRampRate(Preferences.getDouble("Ramp", openRampRate));
    //rightSpark1.setClosedLoopRampRate(Preferences.getDouble("Ramp", closedRampRate));
    rightSpark1.setOpenLoopRampRate(Preferences.getDouble("Ramp", openRampRate));
    //rightSpark2.setClosedLoopRampRate(Preferences.getDouble("Ramp", closedRampRate));
    rightSpark2.setOpenLoopRampRate(Preferences.getDouble("Ramp", openRampRate));

    leftPID = leftSpark1.getPIDController();
    leftPID.setOutputRange(-1.0, 1.0);
    rightPID = rightSpark1.getPIDController();
    rightPID.setOutputRange(-1.0, 1.0);
    
    leftEnc = leftSpark1.getEncoder();
    leftEnc.setPositionConversionFactor(18.84 / 7.31);
    rightEnc = rightSpark1.getEncoder();
    rightEnc.setPositionConversionFactor(18.84 / 7.31);

    leftSpark2.follow(leftSpark1);
    rightSpark2.follow(rightSpark1);
    leftSpark1.setInverted(true);

    drivetrainGyro = new AHRS(SerialPort.Port.kUSB);
    //drivetrainGyro.setSensitivity(Constants.GYRO_kVoltsPerDegreePerSecond);
    drivetrainGyro.calibrate();

    differentialDrive = new DifferentialDrive(rightSpark1, leftSpark1);

    m_odometry = new DifferentialDriveOdometry(drivetrainGyro.getRotation2d());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Drivetrain Left", leftEnc.getPosition());
    SmartDashboard.putNumber("Drivetrain Right", rightEnc.getPosition());
    SmartDashboard.putNumber("P", rightPID.getP());
    SmartDashboard.putNumber("I", rightPID.getI());
    SmartDashboard.putNumber("D", rightPID.getD());
    SmartDashboard.putNumber("Gyro Pos", drivetrainGyro.getAngle());
    SmartDashboard.putNumber("Gyro Rate", drivetrainGyro.getRate());
    m_odometry.update(
        drivetrainGyro.getRotation2d(), leftEnc.getPosition(), rightEnc.getPosition());
  }

  public void setMotors(double left, double right) {
    leftSpark1.set(left);
    rightSpark1.set(right);
  }

  public double encoderToDistanceInch(double rotations) {
    return rotations * (18.84 / 7.31);
  }

  public double getLeftEnc() {
    return leftEnc.getPosition();
  }

  public double getRightEnc() {
    return rightEnc.getPosition();
  }

  public void resetEncoders() {
    leftEnc.setPosition(0.0);
    rightEnc.setPosition(0.0);
  }

  public void setPID(double p, double i, double d, double Iz) {
    //Left
    leftPID.setP(p);
    leftPID.setI(i);
    leftPID.setD(d);
    leftPID.setIZone(Iz);
    //Right
    rightPID.setP(p);
    rightPID.setI(i);
    rightPID.setD(d);
    rightPID.setIZone(Iz);
  }

  public void setPIDReference(double leftReference, double rightReference, ControlType controlType) {
    leftPID.setReference(leftReference, controlType);
    rightPID.setReference(rightReference, controlType);
  }

  public void arcadeDrive(double moveSpeed, double rotateSpeed) {
    differentialDrive.arcadeDrive(moveSpeed, rotateSpeed);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEnc.getVelocity()*0.0254, rightEnc.getVelocity()*0.0254);
  }

  public double getHeading() {
    return drivetrainGyro.getYaw();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftSpark1.setVoltage(leftVolts);
    rightSpark1.setVoltage(rightVolts);
    differentialDrive.feed();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, drivetrainGyro.getRotation2d());
  }
}