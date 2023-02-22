package logic;

import frc.robot.math.math;
import frc.robot.math.vec2;
import org.junit.jupiter.api.*;

import static org.junit.jupiter.api.Assertions.*;
import static util.Util.ACCEPTABLE_ERROR;

public class MathTest {
    @Test
    @DisplayName("Testing basic vec2 functionality")
    void testVec2(){
        vec2 v1 = new vec2(2.625, 5.4375);
        assertAll(
            () -> assertEquals("(2.625, 5.4375)", v1.toString()),
            () -> assertEquals(2.625, v1.x()),
            () -> assertEquals(5.4375, v1.y())
        );
    }
    @Test
    @DisplayName("Testing vec2 operations")
    void testVec2Operations(){
        vec2 v1 = new vec2(2, 5);
        vec2 v2 = new vec2(7, 23);
        double scale = 6;

        assertAll(
            () -> assertEquals("(9.0, 28.0)", v1.plus(v2).toString()),
            () -> assertEquals("(5.0, 18.0)", v2.minus(v1).toString()),
            () -> assertEquals(129, v1.dot(v2)),
            () -> assertEquals("(12.0, 30.0)", v1.times(scale).toString())
        );
    }
    @Test
    @DisplayName("Testing modulo operation")
    void testModulo(){
        assertAll (
            () -> assertEquals(5, math.mod(13, 8)),
            () -> assertEquals(3, math.mod(-13, 8)),
            () -> assertEquals(3, math.mod(23, -10, 10)),
            () -> assertEquals(-8, math.mod(-48, -10, 10))
        );
    }
    @Test
    @DisplayName("Testing vector operations in math")
    void testVecMath(){
        assertAll(
            // Vector magnitude
            () -> assertEquals(5, math.magnitude(3, 4), ACCEPTABLE_ERROR),
            () -> assertEquals(13, math.magnitude(12, 5), ACCEPTABLE_ERROR),

            // Vector normalization
            () -> assertEquals(new vec2(.6, .8).toString(), math.normalize(new vec2(3, 4)).toString()),

            // Vector rotation
            () -> assertEquals(vec2.jHat.toString(), math.rotateCCW(vec2.iHat, 90).toString()),
            () -> assertEquals(vec2.jHat.times(-1).toString(), math.rotateCW(vec2.iHat, 90).toString())
        );
    }
    @Test
    @DisplayName("Testing trig")
    void testTrig(){
        final double SQRT_3 = 1.73205080757;
        final double SQRT_3_2 = SQRT_3 * .5;

        assertAll(
            // Test math.sin
            () -> assertEquals(0, math.sin(0), ACCEPTABLE_ERROR),
            () -> assertEquals(0.5, math.sin(30), ACCEPTABLE_ERROR),
            () -> assertEquals(SQRT_3_2, math.sin(60), ACCEPTABLE_ERROR),
            () -> assertEquals(1, math.sin(90), ACCEPTABLE_ERROR),

            // Test math.cos
            () -> assertEquals(1, math.cos(0), ACCEPTABLE_ERROR),
            () -> assertEquals(SQRT_3_2, math.cos(30), ACCEPTABLE_ERROR),
            () -> assertEquals(0.5, math.cos(60), ACCEPTABLE_ERROR),
            () -> assertEquals(0, math.cos(90), ACCEPTABLE_ERROR),

            // Test math.tan
            () -> assertEquals(0, math.tan(0), ACCEPTABLE_ERROR),
            () -> assertEquals(1, math.tan(45), ACCEPTABLE_ERROR),
            () -> assertEquals(SQRT_3, math.tan(60), ACCEPTABLE_ERROR),

            // Test math.atan2
            () -> assertEquals(0, math.atan(0, 1), ACCEPTABLE_ERROR),
            () -> assertEquals(30, math.atan(1, SQRT_3), ACCEPTABLE_ERROR),
            () -> assertEquals(60, math.atan(SQRT_3, 1), ACCEPTABLE_ERROR),
            () -> assertEquals(90, math.atan(1, 0), ACCEPTABLE_ERROR)
        );
    }
    @Test
    @DisplayName("Testing misc math operations")
    void testMisc(){
        assertAll(
            () -> assertEquals(0.3125, math.round(0.33, 0.0625)),
            () -> assertEquals(0.375, math.round(0.35, 0.0625))
        );
    }
}
