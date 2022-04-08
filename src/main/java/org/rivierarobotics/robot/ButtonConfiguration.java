/*
 * This file is part of MrT-2022, licensed under the GNU General Public License (GPLv3).
 *
 * Copyright (c) Riviera Robotics <https://github.com/Team5818>
 * Copyright (c) contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

package org.rivierarobotics.robot;

import com.ctre.phoenix.led.TwinkleAnimation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.rivierarobotics.commands.advanced.climb.ClimbInterruptToggle;
import org.rivierarobotics.commands.advanced.climb.RetryClicker;
import org.rivierarobotics.commands.advanced.climb.RunClimb;
import org.rivierarobotics.commands.advanced.collect.CollectBalls;
import org.rivierarobotics.commands.advanced.drive.AngulationToTargetBasedOffOfPose;
import org.rivierarobotics.commands.advanced.drive.DrivePathPlannerPath;
import org.rivierarobotics.commands.advanced.drive.DriveToClosest;
import org.rivierarobotics.commands.advanced.shoot.*;
import org.rivierarobotics.commands.auto.*;
import org.rivierarobotics.commands.basic.climb.ClimbSetAngle;
import org.rivierarobotics.commands.basic.climb.ClimbSetVoltage;
import org.rivierarobotics.commands.basic.climb.IdleMode;
import org.rivierarobotics.commands.basic.climb.SetPiston;
import org.rivierarobotics.commands.basic.collect.ToggleIntakeState;
import org.rivierarobotics.commands.basic.drive.SetCameraCentric;
import org.rivierarobotics.commands.basic.drive.SetDriveAngle;
import org.rivierarobotics.commands.basic.drive.SetDriverAssist;
import org.rivierarobotics.commands.basic.drive.SetWheelbaseAngle;
import org.rivierarobotics.commands.basic.shoot.SetFloppaPosition;
import org.rivierarobotics.commands.basic.shoot.SetFloppaZero;
import org.rivierarobotics.commands.basic.shoot.SetFlywheelSpeed;
import org.rivierarobotics.subsystems.climb.Climb;
import org.rivierarobotics.subsystems.climb.ClimbPositions;
import org.rivierarobotics.subsystems.shoot.FloppaFlywheels;
import org.rivierarobotics.subsystems.shoot.ShooterLocations;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;
import org.rivierarobotics.subsystems.vision.Limelight;

public class ButtonConfiguration {
    public void initTeleop() {
        //Driver Left
        new JoystickButton(ControlMap.DRIVER_LEFT, 1)
                .toggleWhenPressed(new ClimbSetAngle(0, false));
        new JoystickButton(ControlMap.DRIVER_LEFT, 2)
                .toggleWhenPressed(new InstantCommand(() -> {
                    DriveTrain.getInstance().setTargetRotationAngle(135);
                }));
        //Driver Right
        new JoystickButton(ControlMap.DRIVER_RIGHT, 1)
                .whenPressed(new ToggleIntakeState());
        new JoystickButton(ControlMap.DRIVER_RIGHT, 2)
                .toggleWhenPressed(new CollectBalls());

        //Driver Buttons
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 1)
                .whenPressed(new AutoAimShootEject());
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 2)
                .whenPressed(new AutoAimShoot());
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 3)
                .whenPressed(new FenderShot());
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 7)
                .whenPressed(new ClimbInterruptToggle());
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 8)
                .whenPressed(new ClimbSetVoltage(false, 0));
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 9)
                .whenPressed(new RunClimb(false));
//        new JoystickButton(ControlMap.DRIVER_BUTTONS, 11)
//                .whenPressed(new RunClimb(true));

        new JoystickButton(ControlMap.DRIVER_BUTTONS, 13)
                .whileHeld(new SetDriverAssist(true));
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 13)
                .whenReleased(new SetDriverAssist(false));

//        new JoystickButton(ControlMap.DRIVER_BUTTONS, 15)
//                .whileHeld(new SetCameraCentric());

        //CO-DRIVER

        //CO-DRIVER JOYSTICK
//        new JoystickButton(ControlMap.CO_DRIVER_LEFT, 1)
//                .whenPressed(new RunClimb(false));

        //CO-DRIVER BUTTONS

        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 1)
                .whenPressed(new SetFloppaZero());
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 2)
//                .whenPressed(new SetDriveAngle(-180));
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 2)
//                .whenPressed(new SetDriveAngle(180));
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 3)
//                .whileHeld(new SetFlywheelSpeed(0));
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 4)
//                .whenPressed(new SetFlywheelSpeed(FloppaFlywheels.getInstance().getTargetVelocity() - 200));
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 5)
//                .whenPressed(new SetFlywheelSpeed(FloppaFlywheels.getInstance().getTargetVelocity() + 200));
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 6)
//                .whenPressed(new CrazyWildCollectionBouncyHousePath());

//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 1)
//                .whenPressed(new SetPiston(ClimbPositions.LOW, true));
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 2)
//                .whenPressed(new SetPiston(ClimbPositions.LOW, false));
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 3)
//                .whenPressed(new SetPiston(ClimbPositions.MID, true));
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 4)
//                .whileHeld(new SetPiston(ClimbPositions.MID, false));
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 5)
//                .whenPressed(new SetPiston(ClimbPositions.HIGH, true));
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 7)
//                .whenPressed(new CrazyWildCollectionBouncyHousePath());


        //new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 8)
        //    .whileHeld(new SequentialCommandGroup(new SetDriveAngle(77)));
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 8).whenPressed(
//                    new InstantCommand(() -> {
//                        DriveTrain.getInstance().getPoseEstimator().resetPose(Limelight.getInstance().getLLAbsPose());
//                    })
//        );
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 9).whenPressed(() -> Climb.getInstance().resetZeros(false));
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 10).whenPressed(new RotateBall());
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 11).whenPressed(new FindBall(true));
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 12).whenPressed(new MLCollect1(false));
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 15)
//                .whileHeld(new TrackGoal());

    }
}
