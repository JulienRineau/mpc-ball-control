import React, { Component } from "react";
import Cards from "./Cards";
import { Container } from "react-bootstrap";
import Video from "./content/Video";
import ProjectOverview from "./content/ProjectOverview";

class Home extends Component {
  state = {};
  render() {
    return (
      <div>
        <div class="p-5 mb-4 bg-light rounded-3">
          <div class="container-fluid py-5">
            <h1 class="display-5 fw-bold">MPC Ball Control</h1>
            <p class="col-md-8 fs-4">
              This project as aim to control a ball on a plate using a robotic
              manipulator and a MPC controller. This project was carried out as
              part of the CS206B course at UC Berkeley
            </p>
            <button
              href="https://github.com/JulienRineau/ball-tracking"
              class="btn btn-dark btn-lg"
              type="button"
            >
              GitHub
            </button>
          </div>
        </div>
        <Container>
          <div>
            <Video />
          </div>

          <hr class="featurette-divider" />
          <div>
            <ProjectOverview />
          </div>
          <hr class="featurette-divider" />

          <div>
            <Cards />
          </div>
          {/* <div>
            <Featurettes />
          </div> */}
        </Container>
      </div>
    );
  }
}

export default Home;
