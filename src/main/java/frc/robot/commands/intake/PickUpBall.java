// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.queuing.QueuingOn;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Queuing;

public class PickUpBall extends CommandBase {
  private Intake intake = null;
  private Queuing queuing = null;
  private boolean ballIn = false;
  private boolean ballQueued = false; 

  /** Creates a new PickUpBall. */
  public PickUpBall(Intake m_intake, Queuing m_queuing) {
    intake = m_intake;
    queuing = m_queuing;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake, m_queuing);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ballIn = false;
    ballQueued = false;
    intake.intakeOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intake.ballInIntake() && ballIn == false) {
      intake.intakeOff();
      queuing.turnQueuingOn();
      ballIn = true;
    } else if (!intake.ballInIntake() && ballIn == true) {
      queuing.turnQueuingOff();
      ballQueued = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ballIn == true && ballQueued == true;
  }
}
