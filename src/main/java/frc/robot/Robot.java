// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {
  private final XboxController m_operator;
  private final DifferentialDrive m_robotDrive;

  private final SparkMax m_leftMotor1 = new SparkMax(1,MotorType.kBrushed);
  private final SparkMax m_leftMotor2 = new SparkMax(2,MotorType.kBrushed);

  private final SparkMax m_rightMotor1 = new SparkMax(3,MotorType.kBrushed);
  private final SparkMax m_rightMotor2 = new SparkMax(4,MotorType.kBrushed);

  private final SparkMax m_shooterIntake = new SparkMax(6,MotorType.kBrushless);
  private final SparkMax m_feeder = new SparkMax(5,MotorType.kBrushless);

  //private final SparkMax m_climber = new SparkMax(7,MotorType.krushed);

  private final SparkMaxConfig c_RM1 = new SparkMaxConfig();
  private final SparkMaxConfig c_LM1 = new SparkMaxConfig();
  private final SparkMaxConfig c_RM2 = new SparkMaxConfig();
  private final SparkMaxConfig c_LM2 = new SparkMaxConfig();

  private double driveMult = 0.85;
  private double driveSteerMult = 0.85;
  private double shooterIntakeMult = 0.7;
  private double feedMult = 0.70;
  private double powerShotMult = 0.95;
  private double powerFeedMult = 0.90;

  /** Called once at the beginning of the robot program. */
  public Robot() {
    c_LM1.inverted(true);
    c_LM2.inverted(true);
    
    c_RM2.follow(m_rightMotor1);
    c_LM2.follow(m_leftMotor1);

    m_rightMotor1.configure(c_RM1, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
    m_leftMotor1.configure(c_LM1, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
    m_rightMotor2.configure(c_RM2, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
    m_leftMotor2.configure(c_LM2, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
    
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_robotDrive =  new DifferentialDrive(m_leftMotor1::set, m_rightMotor1::set);
    m_operator = new XboxController(0);

    SendableRegistry.addChild(m_robotDrive, m_leftMotor1);
    SendableRegistry.addChild(m_robotDrive, m_rightMotor1);
  }

  @Override
  public void teleopPeriodic() {
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    m_robotDrive.arcadeDrive(m_operator.getLeftY() * driveMult, -m_operator.getRightX() * driveSteerMult);

    if(m_operator.getRightTriggerAxis() > 0.5)
    {//Intake/shoot
      m_shooterIntake.set(shooterIntakeMult);
      m_feeder.set(-feedMult);
    }
    else if(m_operator.getYButton())
    {//Shoot to our area/powershot
      m_shooterIntake.set(powerShotMult);
      m_feeder.set(-powerFeedMult);
    }
    else if(m_operator.getLeftTriggerAxis() > 0.5)
    {//Feed
      m_shooterIntake.set(shooterIntakeMult - 0.2);
      m_feeder.set(feedMult);
    }
    else
    {//Stop
      m_shooterIntake.set(0);
      m_feeder.set(0);
    }

    if(m_operator.getXButtonPressed() && m_operator.getBButtonPressed())
    {
        driveMult *= -1;
    }

    /*
    if(m_operator.getYButtonPressed())
    {
      m_climber.set(0.8);
      if(m_operator.getYButtonReleased())
      {
        m_climber.set(0);
      }
    }
    else if(m_operator.getAButton())
    {
      m_climber.set(0.8);
      if(m_operator.getAButtonReleased())
      {
        m_climber.set(0);
      }
    }
    else
    {
      m_climber.set(0);
    }
    */
  }

  @Override
  public void autonomousPeriodic() {
    //new InstantCommand(() -> m_robotDrive.arcadeDrive(0, 0.1)).withTimeout(1.0);
    m_shooterIntake.set(shooterIntakeMult);
    m_feeder.set(-feedMult);
  }

  public void autoCommand()
  {
    new InstantCommand(() -> m_robotDrive.arcadeDrive(0, 0.1)).withTimeout(1.0);
    new InstantCommand(() -> shootInAuto()).withTimeout(10.0);
  }

  public void shootInAuto()
  {
    m_shooterIntake.set(shooterIntakeMult);
    m_feeder.set(-feedMult);
  }


}
