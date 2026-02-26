// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.shooterSubsystem;
import frc.robot.subsystems.turretSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;


/** An example command that uses an example subsystem. */
public class shoot extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final shooterSubsystem m_shooterSubsystem;
  private final turretSubsystem m_turretSubsystem;

  private boolean m_isFinished = false;
  private int shootFlag = 0;
  
    /**
     * Creates a new set-PowerCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public shoot(shooterSubsystem shooter, turretSubsystem hood) {
      m_shooterSubsystem = shooter;
      m_turretSubsystem = hood;
  
      addRequirements(shooter);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      shootFlag = 1;
            m_shooterSubsystem.setShooterSpinSpeed(Constants.kShooterOutSpeed);

    }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (shootFlag) {
        case 1:
          m_shooterSubsystem.setIndexerSpinMotor(Constants.kIndexerMainSpeed);
          shootFlag = 2;
          break;
        case 2:
          if (m_shooterSubsystem.getIndexerSpeed() > 9) {
            m_turretSubsystem.getHoodMotorPos();
            m_shooterSubsystem.setShooterSpinSpeed(Constants.kShooterOutSpeed);
            // some sort of regression line equation to come up with the speed
          }
          break;
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  m_shooterSubsystem.setShooterSpinSpeed(0);
}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
