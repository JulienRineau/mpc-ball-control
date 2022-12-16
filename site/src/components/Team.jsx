import React, { Component } from "react";
import Container from "react-bootstrap/Container";
import Row from "react-bootstrap/Row";
import Col from "react-bootstrap/Col";
import Button from "react-bootstrap/Button";
import Card from "react-bootstrap/Card";

class Team extends Component {
  state = {};
  render() {
    return (
      <div>
        <div class="p-5 mb-4 bg-light rounded-3">
          <div class="container-fluid py-5">
            <h1 class="display-5 fw-bold">Team composition</h1>
            <p class="col-md-8 fs-4">
              The team is composed of 4 students in Master of Engineering in
              Mechanical Engineering at UC Berkeley
            </p>
          </div>
        </div>
        <Container>
          <h1>Team</h1>
          <Row>
            <Col>
              {" "}
              <Card style={{ width: "18rem" }}>
                <Card.Img variant="top" src="holder.js/100px180" />
                <Card.Body>
                  <Card.Title>Chet Kruse</Card.Title>
                  <Card.Text>
                    Chet is a Master’s of Engineering student studying
                    Mechanical Engineering with a focus on Robotics and
                    Autonomous Systems. With a background in controls
                    programming and AI, his main focus for this project was
                    navigating the control strategy for MPC design.
                  </Card.Text>
                  <Button variant="primary">LinkedIn</Button>
                </Card.Body>
              </Card>
            </Col>
            <Col>
              {" "}
              <Card style={{ width: "18rem" }}>
                <Card.Img variant="top" src="holder.js/100px180" />
                <Card.Body>
                  <Card.Title>Alex Rivero</Card.Title>
                  <Card.Text>
                    Alex is Master’s of Engineering student at UC Berkeley
                    studying Mechanical Engineering with a focus on product
                    design and manufacturing processes. With a background in
                    rapid prototyping and mechanical design, his main focus this
                    project was the hardware design as well as helping in the
                    creation of the ball detection node to track the ball.
                  </Card.Text>
                  <Button variant="primary">LinkedIn</Button>
                </Card.Body>
              </Card>
            </Col>
            <Col>
              {" "}
              <Card style={{ width: "18rem" }}>
                <Card.Img variant="top" src="holder.js/100px180" />
                <Card.Body>
                  <Card.Title>Julien Rineau</Card.Title>
                  <Card.Text>
                    As a Master Student at UC Berkeley in Mechanical
                    Engineering, with a focus on robotics, Julie has developed a
                    strong expertise in deep learning, model predictive control,
                    and computer vision. In this current project, he was
                    focusing specifically on computer vision and global software
                    architecture.
                  </Card.Text>
                  <Button variant="primary">LinkedIn</Button>
                </Card.Body>
              </Card>
            </Col>
            <Col>
              {" "}
              <Card style={{ width: "18rem" }}>
                <Card.Img variant="top" src="holder.js/100px180" />
                <Card.Body>
                  <Card.Title>Card Title</Card.Title>
                  <Card.Text>
                    Lorem ipsum dolor sit amet, consectetur adipiscing elit. Sed
                    non risus. Suspendisse lectus tortor, dignissim sit amet,
                    adipiscing nec, ultricies sed, dolor. Cras elementum
                    ultrices diam. Maecenas ligula massa, varius a, semper
                    congue, euismod non, mi. Proin porttitor, orci nec nonummy
                    molestie, enim est eleifend mi, non fermentum diam nisl sit
                    amet erat. Duis semper. Duis arcu massa, scelerisque vitae,
                    consequat in, pretium a, enim.
                  </Card.Text>
                  <Button variant="primary">LinkedIn</Button>
                </Card.Body>
              </Card>
            </Col>
          </Row>
        </Container>
      </div>
    );
  }
}

export default Team;
