// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoShoot;

import frc.robot.subsystems.shooterSubsystem;
import frc.robot.subsystems.turretSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.spiralRollerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.Optional;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;


/** An example command that uses an example subsystem. */
public class onthemove extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final shooterSubsystem m_shooterSubsystem;
  private final turretSubsystem m_turretSubsystem;
  private final spiralRollerSubsystem m_spiralRollerSubsystem;
  private final CommandSwerveDrivetrain m_Drivetrain;
  
  
    private boolean m_isFinished = false;
    private int shootFlag = 0;
    private PoseEstimate m_robotPose;
    private Optional<Alliance> m_alliance;
    private double legOne;
    private double legTwo;
    private double dist;
    private double currentTime;
    private double prevTime;
    private double[] acceleration;
    private ChassisSpeeds velocity;
    private ChassisSpeeds previousLoopVelocity;

    Timer loopTimer = new Timer();
  
    
      /**
       * Creates a new set-PowerCommand.
       *
       * @param subsystem The subsystem used by this command.
       */
      public onthemove(shooterSubsystem shooter, turretSubsystem hood, spiralRollerSubsystem spirals, CommandSwerveDrivetrain drivetrain) {
        m_shooterSubsystem = shooter;
        m_turretSubsystem = hood;
        m_spiralRollerSubsystem = spirals;
        m_Drivetrain = drivetrain;
    
      addRequirements(shooter);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      shootFlag = 1;

      loopTimer.reset();
      loopTimer.start();
      currentTime = loopTimer.get();
      velocity = m_Drivetrain.getState().Speeds;
    }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    previousLoopVelocity = velocity;
    velocity = m_Drivetrain.getState().Speeds;

    prevTime = currentTime;
    currentTime = loopTimer.get();

    acceleration[0] = (velocity.vxMetersPerSecond - previousLoopVelocity.vxMetersPerSecond) / (currentTime - prevTime);
    acceleration[1] = (velocity.vyMetersPerSecond - previousLoopVelocity.vyMetersPerSecond) / (currentTime - prevTime); 

    m_alliance = DriverStation.getAlliance();
    if (m_alliance.get() == Alliance.Red) {
      m_robotPose = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight");
      legTwo = (Constants.kRedHubCoord[0] - m_robotPose.pose.getX());
      legOne = (Constants.kRedHubCoord[1] - m_robotPose.pose.getY());
    } 
    else {
      m_robotPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      legTwo = (Constants.kBlueHubCoord[0] - m_robotPose.pose.getX());
      legOne = (Constants.kBlueHubCoord[1] - m_robotPose.pose.getY());
    }

    dist = Math.sqrt(Math.pow(legTwo, 2) + Math.pow(legOne, 2));

    legTwo = (legTwo - Constants.kShotTimeTable.get(dist)*((acceleration[0]*Constants.kAccelCompFactor) + velocity.vxMetersPerSecond));
    legOne = (legOne - Constants.kShotTimeTable.get(dist)*((acceleration[1]*Constants.kAccelCompFactor) + velocity.vyMetersPerSecond));
   
    dist = Math.sqrt(Math.pow(legTwo, 2) + Math.pow(legOne, 2));

    switch (shootFlag) {
        case 1:
            m_spiralRollerSubsystem.setSpiralRollerSpinSpeed(Constants.kIndexerMainSpeed);
            m_turretSubsystem.setHoodMotor(Constants.kHoodTable.get(dist));
            shootFlag = 2;
          break;
        case 2:
            m_shooterSubsystem.setShooterSpinSpeed(Constants.kSpeedTable.get(dist));
            shootFlag = 3;
          break;
        case 3:
          if (m_shooterSubsystem.getShooterSpeed() > 5) {
            m_shooterSubsystem.setIndexerSpinSpeed(Constants.kIndexerMainSpeed);
          }
          break;
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  m_shooterSubsystem.setShooterSpinSpeed(0);
  m_spiralRollerSubsystem.setSpiralRollerSpinSpeed(0);
  m_shooterSubsystem.setIndexerSpinSpeed(0);
}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
