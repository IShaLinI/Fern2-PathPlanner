// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.util.controls.Deadbander;

public class RobotContainer {

  private final Drivetrain mDrivetrain = new Drivetrain();
  private final Intake mIntake = new Intake();
  private final Pivot mPivot = new Pivot();

  private AutoCommands mAutoCommands = new AutoCommands(mDrivetrain, mIntake, mPivot);

  private CommandXboxController mDriver = new CommandXboxController(0);
  private CommandXboxController mOperator = new CommandXboxController(1);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    //#region Driver Controls
    mDrivetrain.setDefaultCommand(
      new RunCommand(()-> 
        mDrivetrain.drive(
          Deadbander.applyLinearScaledDeadband(-mDriver.getLeftY(),0.1) * DriveConstants.kMaxSpeed,
          Deadbander.applyLinearScaledDeadband(-mDriver.getRightX(), 0.1) * DriveConstants.kMaxTurnSpeed
        ),
        mDrivetrain
      )
    );
    
    mDriver.povUp().onTrue(
      mDrivetrain.changeState(DriveConstants.DirState.FORWARD)
    );

    mDriver.povDown().onTrue(
      mDrivetrain.changeState(DriveConstants.DirState.REVERSE)
    );

    mDriver.leftTrigger().onTrue(
      mDrivetrain.changeState(DriveConstants.SpeedState.TURBO)
    ).onFalse(
      mDrivetrain.changeState(DriveConstants.SpeedState.NORMAL)
    );

    mDriver.rightTrigger().onTrue(
      mDrivetrain.changeState(DriveConstants.SpeedState.SLOW)
    ).onFalse(
      mDrivetrain.changeState(DriveConstants.SpeedState.NORMAL)
    );

    mDriver.rightBumper().onTrue(
      mAutoCommands.getEvent("FireL1")
    ).onFalse(
      mAutoCommands.getEvent("Carry")
    );

    mDriver.leftBumper().onTrue(
      mAutoCommands.getEvent("FireL1")
    ).onFalse(
      mAutoCommands.getEvent("Carry")
    );

    mDriver.a().onTrue(
      mDrivetrain.RotateTo(0)
    );


    //#endregion
    //#region Operator Controls
    
    mOperator.x().onTrue(
      mAutoCommands.getEvent("SubPickup")
    ).onFalse(
      mAutoCommands.getEvent("Carry")
    );
  
    mOperator.rightBumper().onTrue(
      mAutoCommands.getEvent("FloorPickup")
    ).onFalse(
      mAutoCommands.getEvent("Carry")
    );
    
    mOperator.y().onTrue(
      mAutoCommands.getEvent("FireL1")
    ).onFalse(
      mAutoCommands.getEvent("Carry")
    );

    mOperator.b().onTrue(
      mAutoCommands.getEvent("FireL2")
    ).onFalse(
      mAutoCommands.getEvent("Carry")
    );

    mOperator.a().onTrue(
      mAutoCommands.getEvent("FireL3")
    ).onFalse(
      mAutoCommands.getEvent("Carry")
    );

    //#endregion
  }

  public Command getAutonomousCommand() {
    return mAutoCommands.getAuto();
  }
}