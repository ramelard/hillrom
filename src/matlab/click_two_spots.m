function [] = click_two_spots

import java.awt.*;
import java.awt.event.*;

input('Press Enter to get first location');
m = MouseInfo.getPointerInfo();
ptr1 = m.getLocation();

input('Press Enter to get second location');
m = MouseInfo.getPointerInfo();
ptr2 = m.getLocation();

robot = Robot;
robot.mouseMove(ptr1.x, ptr1.y);
robot.mousePress(InputEvent.BUTTON1_MASK);
robot.mouseRelease(InputEvent.BUTTON1_MASK);

robot = Robot;
robot.mouseMove(ptr2.x, ptr2.y);
robot.mousePress(InputEvent.BUTTON1_MASK);
robot.mouseRelease(InputEvent.BUTTON1_MASK);