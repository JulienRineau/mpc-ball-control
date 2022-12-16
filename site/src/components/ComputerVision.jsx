import React, { Component } from "react";
import { Container } from "react-bootstrap";
import Row from "react-bootstrap/Row";
import Col from "react-bootstrap/Col";
import architecture from "../assets/images/architecture.png";
import cv_flow from "../assets/images/cv_flow.png";
import cv_demo from "../assets/videos/cv_demo.mp4";

class ComputerVision extends Component {
  state = {};
  render() {
    return (
      <div>
        <div class="p-5 mb-4 bg-light rounded-3">
          <div class="container-fluid py-5">
            <h1 class="display-5 fw-bold">Computer Vision</h1>
            <p class="col-md-8 fs-4">
              The team is composed of 4 students in Master of Engineering in
              Mechanical Engineering at UC Berkeley
            </p>
          </div>
        </div>
        <Container>
          <h2>Code architecture</h2>
          <div>
            <p>
              In a program that uses ROS (Robot Operating System) to send ping
              pong ball coordinates to a robotic arm, the overall architecture
              would involve several key components. First, there would be one or
              more sensors, such as a camera, that are used to detect the
              position of the ping pong ball. The sensor data would be processed
              by a ROS node, which would extract the ball's coordinates and
              publish them as a ROS message. Next, there would be a second ROS
              node that is responsible for receiving the ball coordinates and
              computing control inputs for the robotic arm. This node would use
              the ball coordinates to compute the desired position and
              orientation of the arm, and would then publish these control
              inputs as a ROS message. Finally, there would be a third ROS node
              that is responsible for receiving the control inputs and sending
              them to the robotic arm. This node would receive the control
              inputs as a ROS message, and would use them to control the arm's
              motors and drive it to the desired position and orientation.
              Overall, this architecture would allow the sensors, computation,
              and control of the robotic arm to be implemented as separate,
              modular ROS nodes. This would make the system more flexible and
              easier to develop, test, and modify
            </p>
          </div>
          <div>
            <div>
              <img
                src={architecture}
                alt="Equation"
                width="70%"
                height="auto"
              />
            </div>
          </div>
          <h2>Computer Vision</h2>
          <div>
            <p>
              To detect a ping pong ball using OpenCV, the first step is to
              threshold the image. This involves setting a threshold value for
              pixel intensity, and converting the image to black and white based
              on whether each pixel is above or below the threshold. This helps
              to separate the ping pong ball from the background. Next, the
              image is blurred using a Gaussian blur. This helps to smooth out
              the edges of the ping pong ball, making it easier to detect. Then,
              the image is eroded and dilated. Erosion removes pixels from the
              edges of objects in the image, while dilation adds pixels to the
              edges. These operations can help to further separate the ping pong
              ball from the background. Once the image has been pre-processed,
              contour detection is used to identify the ping pong ball in the
              image. This involves finding continuous curves in the image that
              correspond to the edges of the ball. Finally, the centroid
              position of the ping pong ball is estimated using the contour
              information. This allows the position of the ball to be
              determined, which can be used for applications such as ball
              tracking or collision avoidance.
            </p>
          </div>
          <div>
            <Row>
              <Col sm>
                <img src={cv_flow} alt="Equation" width="100%" height="auto" />
              </Col>
              <Col sm>
                <video width="100%" height="auto" controls>
                  <source src={cv_demo} type="video/mp4" />
                </video>
              </Col>
            </Row>
          </div>
        </Container>
      </div>
    );
  }
}

export default ComputerVision;
