import React, { Component } from "react";
import { Navbar } from "react-bootstrap";
import { Nav } from "react-bootstrap";

class Header extends Component {
  state = {};
  render() {
    return (
      <Navbar bg="dark" variant="dark px-5" expand="lg" collapseOnSelect>
        <Navbar.Brand href="#home">MPC Ball Control</Navbar.Brand>
        <Navbar.Toggle aria-controls="basic-navbar-nav" />
        <Navbar.Collapse id="basic-navbar-nav">
          <Nav className="me-auto">
            <Nav.Link href="/">Home</Nav.Link>
            <Nav.Link href="/Team">Team</Nav.Link>
            <Nav.Link href="/Controller">MPC Controller</Nav.Link>
            <Nav.Link href="/ComputerVision">Computer vision</Nav.Link>
            <Nav.Link href="/Hardware">Hardware</Nav.Link>
            <Nav.Link
              href="https://github.com/JulienRineau/ball-tracking"
              target="_blank"
              rel="noopener noreferrer"
            >
              GitHub
            </Nav.Link>
          </Nav>
        </Navbar.Collapse>
      </Navbar>
    );
  }
}

export default Header;
