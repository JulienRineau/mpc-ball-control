import React, { Component } from "react";
import ZoomVideo from "./ZoomVideo";

class ProjectOverview extends Component {
  state = {};
  render() {
    return (
      <div>
        <h2>ProjectOverview</h2>
        <p>
          Our project aimed to develop a control system for a robotic arm to
          manipulate a ball on a plate. The control system employed a model
          predictive controller (MPC), custom hardware and computer vision to
          accurately move the ball to a desired location.
        </p>
        <p>
          {" "}
          From a robotic point of view, knowing how to control a ball on a plate
          is interesting for several reasons. First, it is a simple yet
          non-trivial problem that can be used to test and evaluate control
          algorithms. By controlling the ball's position on the plate, we can
          assess the performance and limitations of the algorithm in a
          controlled environment.Second, the ball-on-plate system has many
          practical applications in robotics. For example, it can be used in
          haptic interfaces, where the ball represents a virtual object that the
          user can interact with using their hands. In this case, the plate
          would be a force feedback device that allows the user to feel the
          forces exerted by the ball. Another example is a ball-balancing robot,
          where the ball represents the robot's center of mass and the plate is
          the robot's base. In this case, the control algorithm would be used to
          maintain the ball's position on the plate and prevent the robot from
          falling over. In both of these examples, the ability to control the
          ball's position on the plate is essential for achieving a realistic
          and stable interaction. MPC is a particularly well-suited algorithm
          for this problem because it allows for precise control and the
          incorporation of constraints and optimization objectives.
        </p>
        <ZoomVideo />
      </div>
    );
  }
}

export default ProjectOverview;
