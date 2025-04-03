// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at https://github.com/Mechanical-Advantage/RobotCode2025Public/blob/main/LICENSE

package tech.lindblom.swerve.utils.mechad;

import edu.wpi.first.math.geometry.Twist2d;

public class EqualsUtil {
  public static boolean epsilonEquals(double a, double b, double epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  public static boolean epsilonEquals(double a, double b) {
    return epsilonEquals(a, b, 1e-9);
  }

  /** Extension methods for wpi geometry objects */
  public static class GeomExtensions {
    public static boolean epsilonEquals(Twist2d twist, Twist2d other) {
      return EqualsUtil.epsilonEquals(twist.dx, other.dx)
          && EqualsUtil.epsilonEquals(twist.dy, other.dy)
          && EqualsUtil.epsilonEquals(twist.dtheta, other.dtheta);
    }
  }
}
