import React, { Component } from "react";
import { Container } from "react-bootstrap";
import Row from "react-bootstrap/Row";
import Col from "react-bootstrap/Col";
import all_it from "../assets/images/all_it.png";
import first_it from "../assets/images/first_it.png";
import sec_it from "../assets/images/sec_it.png";
import last_it from "../assets/images/last_it.png";
import last_it_pic from "../assets/images/last_it_pic.png";

class Hardware extends Component {
  state = {};
  render() {
    return (
      <div>
        <div class="p-5 mb-4 bg-light rounded-3">
          <div class="container-fluid py-5">
            <h1 class="display-5 fw-bold">Hardware</h1>
            <p class="col-md-8 fs-4">
              The team is composed of 4 students in Master of Engineering in
              Mechanical Engineering at UC Berkeley
            </p>
          </div>
        </div>
        <Container>
          <div>
            <p>
              The hardware used for this project was essentially one webcam used
              to track the ping-pong ball and the platform where the ball needed
              to be balanced. The platform needed to be able to get attached to
              the Sawyer arm, which needed to be as light as possible to ensure
              smooth and precise movement from the arm. The following image
              corresponds to the first design of the platform. The primary
              materials used in this first version were: PLA ⅛ in, Plywood Sheet
              and Wooden square profiles. The manufacturing techniques were 3D
              printing as well as laser cutting.
            </p>
          </div>
          <div>
            <Row>
              <Col sm>
                {" "}
                <div>
                  <img
                    src={first_it}
                    alt="Equation"
                    width="50%"
                    height="auto"
                  />
                </div>
              </Col>
              <Col sm>
                <div>
                  <img
                    src={first_it}
                    alt="Equation"
                    width="50%"
                    height="auto"
                  />
                </div>
              </Col>
            </Row>
          </div>
          <div>
            <p>
              After manufacturing and assembling this first version we did a
              first balancing test with the platform attached to the robot as
              can be seen in the video above. Even if at that time neither the
              controller nor the tracking system was finished, we clearly
              identified a serious problem with this design. The problem was
              that the platform was too wobbly, which caused the camera to
              shake. Therefore, tracking and control of the ball were really
              difficult in those conditions. The design of the second iteration
              of the platform is depicted in the following image.
            </p>
          </div>
          <div>
            <div>
              <img src={sec_it} alt="Equation" width="25%" height="auto" />
            </div>
          </div>
          <div>
            <p>
              The two major modifications from the first to the second version
              were that now the critical 3D-printed pieces were printed with a
              higher infill. The positive thing about this change was that now
              these pieces were considerably stronger, however, the downside was
              that now they were heavier. On the other hand, the second major
              modification was that the attachment position for the robot arm
              changed to an upper-centered position. The idea behind this
              decision was to have a more equilibrated position allowing the
              robot to move with less effort. However, this did not work either,
              the problem was persisting, and the platform and camera were
              shaking too much every time the robot moved. We needed to design
              and build a platform that was rigid enough to avoid vibrations but
              at the same time it needed to be lightweight, otherwise, the robot
              wouldn’t be able to control the platform with precision.
            </p>
          </div>
          <div>
            <img src={all_it} alt="Equation" width="50%" height="auto" />
          </div>
          <div>
            <p>
              Attempt after attempt, a total of six different designs were made.
              The biggest modification was that now we were not using any more
              wooden square profiles, we switched to carbon fiber. With the
              final version, we achieved a platform that was rigid enough to
              keep the camera from shaking, all this while keeping it
              lightweight.
            </p>
          </div>
          <div>
            <Row>
              <Col sm>
                {" "}
                <div>
                  <img src={last_it} alt="Equation" width="50%" height="auto" />
                </div>
              </Col>
              <Col sm>
                {" "}
                <div>
                  <img
                    src={last_it_pic}
                    alt="Equation"
                    width="50%"
                    height="auto"
                  />
                </div>
              </Col>
            </Row>
          </div>
        </Container>
      </div>
    );
  }
}

export default Hardware;
