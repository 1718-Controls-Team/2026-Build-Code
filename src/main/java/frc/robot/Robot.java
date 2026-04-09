// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.shooterIndexer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeFuel;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private final intakeFuel m_intakeSubsystem;
  private final shooterIndexer m_shooterSubsystem;
  LimelightHelpers.PoseEstimate llMeasurementTurret;
  LimelightHelpers.PoseEstimate llMeasurementShooter;
  private boolean kUseLimelight = true;
  private String allianceString; 
  private Optional<Alliance> m_alliance;

  public Robot() {
    m_robotContainer = new RobotContainer();
    m_intakeSubsystem = new intakeFuel();
    m_shooterSubsystem = new shooterIndexer();

    int[] validIDs = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32};
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight-lime", validIDs);

    RobotController.setBrownoutVoltage(Constants.kCustomBrownout);
  }

  @Override
  public void robotPeriodic() {
    
    CommandScheduler.getInstance().run(); 
    SmartDashboard.putNumber("Pigeon", m_robotContainer.drivetrain.getPigeon2().getRotation2d().getDegrees());
    SmartDashboard.putNumber("velocity", m_shooterSubsystem.getShooterSpeed());
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if(alliance.get() == Alliance.Red){
      allianceString = "Red";
    }
    else if(alliance.get() == Alliance.Blue){
      allianceString = "Blue";
    }
    SmartDashboard.putString("Alliance", allianceString);


    // limelight stuff down below 

    
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
      LimelightHelpers.SetIMUMode("limelight-cool", 1);

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
    LimelightHelpers.setCameraPose_RobotSpace("limelight", Constants.kLLForwardPos.get(m_intakeSubsystem.getIntakeElectricSlidePos()), 0, 0.47, 0, 0, 0);
    LimelightHelpers.setCameraPose_RobotSpace("limelight-cool", -0.33, 0, 0.29, 0, 5, headingDeg);
   
   
    if (kUseLimelight) {
      LimelightHelpers.SetIMUMode("limelight-cool", 4);
      LimelightHelpers.SetIMUAssistAlpha("limelight-cool", 0.001);
      LimelightHelpers.SetRobotOrientation("limelight", headingDeg, 0,0,0,0,0);
      LimelightHelpers.SetRobotOrientation("limelight-cool", headingDeg, 0,0,0,0,0);
    
      llMeasurementTurret = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      llMeasurementShooter = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-cool");


     if (LimelightHelpers.getTV("limelight")) {

        if (llMeasurementTurret != null && llMeasurementTurret.tagCount > 0 && (m_robotContainer.drivetrain.getState().Speeds.omegaRadiansPerSecond < 1.5)) {
          m_robotContainer.drivetrain.addVisionMeasurement(llMeasurementTurret.pose, Utils.fpgaToCurrentTime(llMeasurementTurret.timestampSeconds),VecBuilder.fill(0.1, 0.1, 0.1));
          m_robotContainer.drivetrain.setStateStdDevs(VecBuilder.fill(999, 999, 0.1));
        }
      }

      if (LimelightHelpers.getTV("limelight-cool")) {

        if (llMeasurementShooter != null && llMeasurementShooter.tagCount > 0 && (m_robotContainer.drivetrain.getState().Speeds.omegaRadiansPerSecond < 1.5)) {
          m_robotContainer.drivetrain.addVisionMeasurement(llMeasurementShooter.pose, Utils.fpgaToCurrentTime(llMeasurementShooter.timestampSeconds),VecBuilder.fill(0.1, 0.1, 0.1));
          m_robotContainer.drivetrain.setStateStdDevs(VecBuilder.fill(99, 99, 0.1));
        }
      }
    } 
    SmartDashboard.putNumber("robot heading", headingDeg);
    SmartDashboard.putNumber("robot Y", m_robotContainer.drivetrain.getState().Pose.getY());
    SmartDashboard.putNumber("robot X", m_robotContainer.drivetrain.getState().Pose.getX());
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
