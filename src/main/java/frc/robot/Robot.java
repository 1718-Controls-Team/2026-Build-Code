// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  LimelightHelpers.PoseEstimate LLmeasurementTurret;

  public Robot() {
    m_robotContainer = new RobotContainer();

    int[] validIDs = {5, 8, 9, 10, 11, 15, 16, 18, 21, 24, 25, 26, 31, 32};
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight-lime", validIDs);
  }

  @Override
  public void robotPeriodic() {
    double headingDeg = m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees();

    LimelightHelpers.SetRobotOrientation("limelight-lime", headingDeg, 0,0,0,0,0);
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    SmartDashboard.putNumber("Pigeon", m_robotContainer.drivetrain.getPigeon2().getRotation2d().getDegrees());

  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
