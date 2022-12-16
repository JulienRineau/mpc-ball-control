import React, { Component } from "react";
import { Route, BrowserRouter as Router, Routes } from "react-router-dom";
import Home from "./Home";
import Team from "./Team";
import Controller from "./Controller";
import ComputerVision from "./ComputerVision";
import Hardware from "./Hardware";

class Body extends Component {
  state = {};
  render() {
    return (
      <Router>
        <Routes>
          <Route path="/" exact element={<Home />} />
          <Route path="/Team" exact element={<Team />} />
          <Route path="/Controller" exact element={<Controller />} />
          <Route path="/ComputerVision" exact element={<ComputerVision />} />
          <Route path="/Hardware" exact element={<Hardware />} />
        </Routes>
      </Router>
    );
  }
}

export default Body;
