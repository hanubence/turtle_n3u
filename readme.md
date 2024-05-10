# Turtlesim fractal
Course assignment for `Advancement robot programming in ROS` course at __Óbuda University__

![image](https://github.com/hanubence/turtle_n3u/assets/32911312/b13a17a5-3ea7-4f51-9e5f-a88bc44e2462)

# PTController for Turtlesim

The PTController is a ROS 2 Node developed to control a Turtlesim turtle in drawing complex geometric patterns such as the Koch Snowflake and triangles. This controller uses proportional control for both linear and angular movements to achieve precise positioning.

### Features:

- Proportional Distance Control: Manages the movement of the turtle with a proportional gain to minimize error in reaching target points.
- Proportional Angular Control: Adjusts the turtle's orientation with proportional control to achieve precise angular positioning.
- Koch Snowflake Drawing: Recursive function to draw a Koch Snowflake with configurable size and depth.
- Triangle Drawing: Functionality to draw an equilateral triangle with a specified side length.