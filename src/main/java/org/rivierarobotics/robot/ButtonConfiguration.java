/*
 * This file is part of Placeholder-2021, licensed under the GNU General Public License (GPLv3).
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

import edu.wpi.first.wpilibj.Joystick;
import org.rivierarobotics.inject.CommandComponent;
import org.rivierarobotics.inject.Input;

import javax.inject.Inject;

public class ButtonConfiguration {
    private final Joystick driverLeft;
    private final Joystick driverRight;
    private final Joystick coDriverLeft;
    private final Joystick coDriverRight;
    private final Joystick driverButtons;
    private final Joystick coDriverButtons;
    private final CommandComponent cmds;

    @Inject
    public ButtonConfiguration(@Input(user = Input.User.DRIVER, type = Input.Type.LEFT_JS) Joystick driverLeft,
                               @Input(user = Input.User.DRIVER, type = Input.Type.RIGHT_JS) Joystick driverRight,
                               @Input(user = Input.User.CODRIVER, type = Input.Type.LEFT_JS) Joystick coDriverLeft,
                               @Input(user = Input.User.CODRIVER, type = Input.Type.RIGHT_JS) Joystick coDriverRight,
                               @Input(user = Input.User.DRIVER, type = Input.Type.BUTTONS) Joystick driverButtons,
                               @Input(user = Input.User.CODRIVER, type = Input.Type.BUTTONS) Joystick coDriverButtons,
                               CommandComponent.Builder component) {
        this.driverLeft = driverLeft;
        this.driverRight = driverRight;
        this.coDriverLeft = coDriverLeft;
        this.coDriverRight = coDriverRight;
        this.driverButtons = driverButtons;
        this.coDriverButtons = coDriverButtons;
        this.cmds = component.build();
    }

    public void initTeleop() {
    }
}
