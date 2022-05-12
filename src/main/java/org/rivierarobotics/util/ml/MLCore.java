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

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonDeserializationContext;
import com.google.gson.JsonDeserializer;
import com.google.gson.JsonElement;
import com.google.gson.JsonParseException;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.rivierarobotics.robot.Logging;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;
import org.rivierarobotics.util.Gyro;
import org.rivierarobotics.util.aifield.FieldMesh;

import java.lang.reflect.Type;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class MLCore {
    private static MLCore INSTANCE;

    public static MLCore getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new MLCore();
        }
        return INSTANCE;
    }

    public static final double ANGLE_PER_PIXEL_X = 1 / 6.0;
    public static final double ANGLE_PER_PIXEL_Y = 1 / 6.0;
    public static final int CAMERA_WIDTH = 640;
    public static final int CAMERA_HEIGHT = 480;
    public static final String TARGET_COLOR = "red";
    public static Trajectory trajectory;

    public static Trajectory getBallTrajectory(DriveTrain driveTrain, Gyro gyro, FieldMesh fieldMesh) {
        var currentPose = driveTrain.getPoseEstimator().getRobotPose();
        var currentX = currentPose.getX();
        var currentY = currentPose.getY();

        MLCore core = MLCore.getInstance();
        var ballColor = DriverStation.getAlliance() == DriverStation.Alliance.Blue ? "blue" : "red";

        var balls = core.getDetectedObjects().get(ballColor);
        if (balls == null || balls.isEmpty()) {
            return null;
        }

        balls.sort(Comparator.comparingDouble(a -> a.relativeLocationDistance));
        for (var ball : balls) {
            var gyroMath = gyro.getRotation2d().getRadians() + Math.toRadians(ball.ty);

            var targetX = currentX + Math.cos(gyroMath) * ball.relativeLocationDistance;
            var targetY = currentY + Math.sin(gyroMath) * ball.relativeLocationDistance;
            Logging.robotShuffleboard.getTab("ML")
                    .setEntry("targetX", targetX)
                    .setEntry("targetY", targetY)
                    .setEntry("currentX", currentX)
                    .setEntry("currentY", currentY);

            var trajectory = fieldMesh.getTrajectory(currentX, currentY, targetX, targetY, true, 0, driveTrain.getSwerveDriveKinematics());
            if (trajectory != null) {
                MLCore.trajectory = trajectory;
                return trajectory;
            }
        }
        return null;
    }

    private static class MLResponseDeserializer implements JsonDeserializer<MLResponse> {
        public MLResponse deserialize(JsonElement json, Type typeOfT, JsonDeserializationContext context) throws JsonParseException {
            MLResponse response = new MLResponse();
            var detectedObjects = json.getAsJsonArray();

            for (var object : detectedObjects) {
                var jsonObject = object.getAsJsonObject();
                var boundingBoxJson = jsonObject.get("box").getAsJsonObject();
                var boundingBox = new BoundingBox(
                        boundingBoxJson.get("ymin").getAsInt(),
                        boundingBoxJson.get("ymax").getAsInt(),
                        boundingBoxJson.get("xmin").getAsInt(),
                        boundingBoxJson.get("xmax").getAsInt()
                );

                var tmp = new MLObject(jsonObject.get("label").getAsString(), boundingBox, jsonObject.get("confidence").getAsDouble());
                response.objects.add(tmp);
            }

            return response;
        }
    }

    private final Gson mlInputParser;

    private MLCore() {
        GsonBuilder builder = new GsonBuilder();
        builder.registerTypeAdapter(MLResponse.class, new MLResponseDeserializer());
        this.mlInputParser = builder.create();
    }

    public Map<String, List<MLObject>> getDetectedObjects() {
        MLResponse resp = getResponse();
        Map<String, List<MLObject>> ret = new HashMap<>();
        if (resp == null) {
            return ret;
        }
        for (var obj : resp.objects) {
            if (!ret.containsKey(obj.label)) {
                ret.put(obj.label, new ArrayList<>());
            }
            ret.get(obj.label).add(obj);
        }
        return ret;
    }

    public MLResponse getResponse() {
        try {
            String mlOut = Logging.networkTableInstance.getTable("ML").getEntry("detections").getString("");
            return mlInputParser.fromJson(mlOut, MLResponse.class);
        } catch (Exception e) {
            e.printStackTrace();
        }
        return new MLResponse();
    }
}
