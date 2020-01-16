package ch.team6417.lib.utils;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import ch.team6417.lib.utils.LatchedBoolean.EdgeDetection;
 
class LatchedBooleanTest {
 
    @Test
    void risingEdgeTest() {
        LatchedBoolean b = new LatchedBoolean(EdgeDetection.RISING);
        Assertions.assertTrue(b.update(true));
        Assertions.assertFalse(b.update(true));
        Assertions.assertFalse(b.update(false));
        Assertions.assertFalse(b.update(false));
        Assertions.assertTrue(b.update(true));
    }

    @Test
    void fallingEdgeTest() {
        LatchedBoolean b = new LatchedBoolean(EdgeDetection.FALLING);
        Assertions.assertFalse(b.update(true));
        Assertions.assertFalse(b.update(true));
        Assertions.assertTrue(b.update(false));
        Assertions.assertFalse(b.update(false));
        Assertions.assertFalse(b.update(true));
    }

    @Test
    void risingFallingEdgeTest() {
        LatchedBoolean b = new LatchedBoolean(EdgeDetection.BOTH);
        Assertions.assertTrue(b.update(true));
        Assertions.assertFalse(b.update(true));
        Assertions.assertTrue(b.update(false));
        Assertions.assertFalse(b.update(false));
        Assertions.assertTrue(b.update(true));
    }
}