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

package org.rivierarobotics.util.AIField;

import java.util.ArrayList;
import java.util.List;


/**
 * Represents a 2D point on the field. Contains a
 * Heuristic function which is used in A* Pathfinding and
 * connections to nearby nodes.
 */
public class FieldNode implements Comparable<FieldNode> {
    boolean isValid = true;

    public FieldNode parent = null;
    double xValue = 0;
    double yValue = 0;
    double nodeWeight = 0;

    public List<Edge> neighbors;

    public double f = Double.MAX_VALUE;
    public double g = Double.MAX_VALUE;

    FieldNode(double xValue, double yValue) {
        this.xValue = xValue;
        this.yValue = yValue;
        this.neighbors = new ArrayList<>();
    }

    @Override
    public int compareTo(FieldNode n) {
        return Double.compare(this.f, n.f);
    }

    public static class Edge {
        Edge(double weight, FieldNode node) {
            this.weight = weight;
            this.node = node;
        }

        public double weight;
        public FieldNode node;
    }

    public void addBranch(double weight, FieldNode node) {
        Edge newEdge = new Edge(weight, node);
        neighbors.add(newEdge);
    }

    public double calculateHeuristic(FieldNode target) {
        double xDist = target.xValue - xValue;
        double yDist = target.yValue - yValue;

        return Math.sqrt(xDist * xDist + yDist * yDist) + nodeWeight;
    }

    @Override
    public String toString() {
        return xValue + " " + yValue;
    }
}
