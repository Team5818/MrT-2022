/*
 * This file is part of Placeholder-2022, licensed under the GNU General Public License (GPLv3).
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

package org.rivierarobotics.util.ml;

public class MLObject {
    public final String label;
    public final BoundingBox boundingBox;
    public final double tx;
    public final double ty;
    public final double confidence;

    public MLObject(String label, BoundingBox boundingBox, double confidence) {
        this.label = label;
        this.boundingBox = boundingBox;

        double avgX = (boundingBox.xmax + boundingBox.xmin) / 2.0;
        tx = (avgX - MLCore.CAMERA_WIDTH / 2.0) * MLCore.ANGLE_PER_PIXEL_X;

        double avgY = (boundingBox.ymax + boundingBox.ymin) / 2.0;
        ty = (avgY - MLCore.CAMERA_HEIGHT / 2.0) * MLCore.ANGLE_PER_PIXEL_Y;

        this.confidence = confidence;
    }
}
