// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.CommandFactory;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.util.controls.Deadbander;

public class RobotContainer {

  private final Drivetrain mDrivetrain = new Drivetrain();
  private final Intake mIntake = new Intake();
  private final Pivot mPivot = new Pivot();

  private CommandFactory mCommandFactory = new CommandFactory(mDrivetrain, mIntake, mPivot);
  private AutoCommands mAutoCommands = new AutoCommands(mDrivetrain, mCommandFactory);

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

    //#endregion
    //#region Operator Controls
    
    mOperator.x().onTrue(
      mCommandFactory.getSubPickup()
    ).onFalse(
      mCommandFactory.getCarry()
    );
  
    mOperator.x().onTrue(
      mCommandFactory.getFloorPickup()
    ).onFalse(
      mCommandFactory.getCarry()
    );
    
    mOperator.x().onTrue(
      mCommandFactory.getFireL1()
    ).onFalse(
      mCommandFactory.getCarry()
    );

    mOperator.x().onTrue(
      mCommandFactory.getFireL2()
    ).onFalse(
      mCommandFactory.getCarry()
    );

    mOperator.x().onTrue(
      mCommandFactory.getFireL3()
    ).onFalse(
      mCommandFactory.getCarry()
    );

    //#endregion
  }

  public Command getAutonomousCommand() {
    return mAutoCommands.getAuto();
  }

  public void stopAll(){
    CommandScheduler.getInstance().schedule(
      new ParallelCommandGroup(
        new InstantCommand(() -> mDrivetrain.stop()),
        new InstantCommand(() -> mIntake.changeState(IntakeConstants.State.STOP)),
        new InstantCommand(() -> mPivot.stop())
      ).ignoringDisable(true)
    );
  }

}