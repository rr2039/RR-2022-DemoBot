package frc.robot.interfaces;

import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Drivetrain extends Subsystem {
    public void arcadeDrive(double moveSpeed, double rotateSpeed);
    public double encoderToDistanceInch(double rotations);
    public double getLeftEnc();
    public double getRightEnc();
    public void resetEncoders();
    public void setMotors(double left, double right);
    public void setPID(double p, double i, double d, double Iz);
    public void setPIDReference(double leftReference, double rightReference, ControlType controlType);
    public DifferentialDriveWheelSpeeds getWheelSpeeds();
    public double getHeading();
    public Pose2d getPose();
    public void tankDriveVolts(double leftVolts, double rightVolts);
    public void resetOdometry(Pose2d pose);
}
