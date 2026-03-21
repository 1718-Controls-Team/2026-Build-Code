// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.shooterIndexer;
import frc.robot.subsystems.intakeFuel;
import frc.robot.subsystems.turretHood;
import frc.robot.subsystems.spiralRoller;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;


/** An example command that uses an example subsystem. */
public class turnTsAround extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final turretHood m_turretSubsystem;

  
  
    private boolean m_isFinished = false;
  
    
      /**
       * Creates a new set-PowerCommand.
       *
       * @param subsystem The subsystem used by this command.
       */
      public turnTsAround(turretHood turret) {
        m_turretSubsystem = turret;
    
      addRequirements(turret);

    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      m_turretSubsystem.setTurretMotorPos(2);
      
    }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
      }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
