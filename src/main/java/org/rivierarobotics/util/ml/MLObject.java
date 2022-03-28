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

package org.rivierarobotics.util.ml;

public class MLObject {
    public static final double BALL_RADIUS = 2 * 0.12;
    public static final double CAMERA_OFFSET = 0.775 - BALL_RADIUS;

    public final String label;
    public final BoundingBox boundingBox;
    public final double ty;
    public final double tx;
    public final double relativeLocationY;
    public final double relativeLocationX;
    public final double relativeLocationDistance;
    public final double confidence;

    public MLObject(String label, BoundingBox boundingBox, double confidence) {
        this.label = label;
        this.boundingBox = boundingBox;
        // Use the bounding box Y value to compute our field X value
        double avgX = (boundingBox.ymax + boundingBox.ymin) / 2.0;
        this.tx = (MLCore.CAMERA_HEIGHT / 2.0 - avgX) * MLCore.ANGLE_PER_PIXEL_Y;
        this.relativeLocationX = Math.tan((90 - Math.abs(tx)) * Math.PI / 180) * CAMERA_OFFSET;

        double avgY = (boundingBox.xmax + boundingBox.xmin) / 2.0;
        this.ty = (MLCore.CAMERA_WIDTH / 2.0 - avgY) * MLCore.ANGLE_PER_PIXEL_X;
        this.relativeLocationY = relativeLocationX * Math.tan(ty * Math.PI / 180);

        this.relativeLocationDistance = Math.sqrt(Math.pow(relativeLocationY, 2) + Math.pow(relativeLocationX, 2));
        this.confidence = confidence;
    }
}
