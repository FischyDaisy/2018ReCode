/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.tracking.GoalTracking;

public class AlignToGoal extends CommandBase {
  /**
   * Creates a new AlignToGoal.
   */
  private GoalTracking mGoalTracker;
  private DriveTrain mDrive;
  private RobotContainer mRobotContainer;
  private DoubleSupplier mLeftY;
  private double kPTurn = 1.0 / 18.0;
  private double kITurn = 0.05;
  private double kDTurn = 0.35;//5;

  private double distance;
  private double speed;
  private double turnError;
  private double turnSlope;
  private double lastTurnError;
  private double turn;
  private double lastTime;

  public AlignToGoal(DriveTrain drive, DoubleSupplier leftY) {
    // Use addRequirements() here to declare subsystem dependencies.
    mGoalTracker = GoalTracking.getInstance();
    mDrive = drive;
    mLeftY = leftY;
    //mRobotContainer = RobotContainer.getInstance();
    addRequirements(mDrive);
    distance = 0.0;
    speed = 0.0;
    turnError = 0.0;
    turnSlope = 0.0;
    turn = 0.0;
    lastTurnError = 0.0;
    lastTime = 0.0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    lastTurnError = turnError;
    if (mGoalTracker.tv > 0.0) {
      turnError = mGoalTracker.tx;
      turnSlope = turnError - lastTurnError;
      turn = turnError * kPTurn + turnSlope * kDTurn; 
      mDrive.setSpeedTurn(mLeftY.getAsDouble(), turn);
    } else {
      mDrive.setSpeedTurn(0.0, 0.0);
    }
    System.out.println("Turn Error: " + turnError);
    System.out.println("Turn Slope: " + turnSlope);
    System.out.println("Turn: " + turn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
