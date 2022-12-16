import React, { Component } from "react";
import Container from "react-bootstrap/Container";
import Row from "react-bootstrap/Row";
import Col from "react-bootstrap/Col";
import demo from "../../assets/videos/demo.mp4";
import real_test_visu from "../../assets/videos/real_test_visu.mp4";

class Video extends Component {
  state = {};
  render() {
    return (
      <div>
        <Container>
          {" "}
          <Row>
            <Col sm>
              {" "}
              <video width="100%" height="auto" controls>
                <source src={demo} type="video/mp4" />
              </video>
            </Col>
            <Col sm>
              {" "}
              <video width="100%" height="auto" controls>
                <source src={real_test_visu} type="video/mp4" />
              </video>
            </Col>
          </Row>
        </Container>
      </div>
    );
  }
}

export default Video;
