import React, { Component } from "react";
import Figure from "react-bootstrap/Figure";
import Container from "react-bootstrap/Container";
import Row from "react-bootstrap/Row";
import Col from "react-bootstrap/Col";

class Results extends Component {
  state = {};
  render() {
    return (
      <div>
        <h2>Results</h2>
        <p>
          Lorem ipsum dolor sit amet, consectetur adipiscing elit. Suspendisse
          rutrum ac ex non dapibus. Sed quis est vehicula, eleifend magna non,
          fringilla sapien. Donec sagittis non odio fermentum interdum. Nunc
          fermentum vulputate libero, et lacinia enim rutrum sit amet. Ut sit
          amet auctor erat. Sed dignissim placerat viverra. Proin neque ipsum,
          faucibus eget elementum vehicula, tempor nec orci.
        </p>
        <Container>
          <Row className="justify-content-md-center">
            <Col sm>
              {" "}
              <Figure>
                <Figure.Image
                  width={171}
                  height={180}
                  alt="171x180"
                  src="holder.js/171x180"
                />
                <Figure.Caption>
                  Nulla vitae elit libero, a pharetra augue mollis interdum.
                </Figure.Caption>
              </Figure>
            </Col>
            <Col sm>
              {" "}
              <Figure>
                <Figure.Image
                  width={171}
                  height={180}
                  alt="171x180"
                  src="holder.js/171x180"
                />
                <Figure.Caption>
                  Nulla vitae elit libero, a pharetra augue mollis interdum.
                </Figure.Caption>
              </Figure>
            </Col>
            <Col sm>
              {" "}
              <Figure>
                <Figure.Image
                  width={171}
                  height={180}
                  alt="171x180"
                  src="holder.js/171x180"
                />
                <Figure.Caption>
                  Nulla vitae elit libero, a pharetra augue mollis interdum.
                </Figure.Caption>
              </Figure>
            </Col>
          </Row>
        </Container>
      </div>
    );
  }
}

export default Results;
