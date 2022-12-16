import React, { Component } from "react";
import Container from "react-bootstrap/Container";
import Row from "react-bootstrap/Row";
import Col from "react-bootstrap/Col";
import ReactPlayer from "react-player/youtube";

class ZoomVideo extends Component {
  state = {};
  render() {
    return (
      <div>
        <Container>
          {" "}
          <Row>
            <Col sm>
              <ReactPlayer url="https://youtu.be/FzEicyJltkM" />;
            </Col>
          </Row>
        </Container>
      </div>
    );
  }
}

export default ZoomVideo;
