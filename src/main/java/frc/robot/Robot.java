// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private Field2d field2d = new Field2d();
  private Field2d odomField = new Field2d();
  LimelightHelpers.PoseEstimate llMeasurementTurret;
  private boolean kUseLimelight = true;

  public Robot() {
    m_robotContainer = new RobotContainer();

    int[] validIDs = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32};
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight-lime", validIDs);

    SmartDashboard.putData(field2d);
    SmartDashboard.putData(odomField);
  }

  @Override
  public void robotPeriodic() {
    
    CommandScheduler.getInstance().run(); 
    SmartDashboard.putNumber("Pigeon", m_robotContainer.drivetrain.getPigeon2().getRotation2d().getDegrees());



    // limelight stuff down below 

    
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {

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
  public void teleopPeriodic() {
    double headingDeg = m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees();

    if (kUseLimelight) {
      
    LimelightHelpers.SetRobotOrientation("limelight", headingDeg, 0,0,0,0,0);
    llMeasurementTurret = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");


    if (LimelightHelpers.getTV("limelight")) {
      var driveState = m_robotContainer.drivetrain.getState();

      field2d.setRobotPose(llMeasurementTurret.pose);
      odomField.setRobotPose(driveState.Pose);
      if (llMeasurementTurret != null && llMeasurementTurret.tagCount > 0 && (m_robotContainer.drivetrain.getState().Speeds.omegaRadiansPerSecond < 1.5)) {
        m_robotContainer.drivetrain.addVisionMeasurement(llMeasurementTurret.pose, Utils.fpgaToCurrentTime(llMeasurementTurret.timestampSeconds),VecBuilder.fill(0.1, 0.1, .1));
      }
    }
  }
    SmartDashboard.putNumber("robot heading", headingDeg);
    SmartDashboard.getNumber("Pigeon", m_robotContainer.drivetrain.getPigeon2().getRotation2d().getDegrees());
  }

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
