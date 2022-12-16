import React, { Component } from "react";
import Navbar from "react-bootstrap/Navbar";

class Footer extends Component {
  state = {};
  render() {
    return (
      <Navbar
        bg="dark"
        variant="dark px-5 mt-5"
        className="justify-content-center"
      >
        <Navbar.Brand>
          Made with love by A.Rivero C.Kruse I.R.Aguirre J.Rineau
        </Navbar.Brand>
      </Navbar>
    );
  }
}

export default Footer;
