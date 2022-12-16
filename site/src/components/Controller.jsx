import React, { Component } from "react";
import { Container } from "react-bootstrap";
import equation from "../assets/images/equation.png";
import simulaton from "../assets/images/tracking_sim.gif";

class Controller extends Component {
  state = {};
  render() {
    return (
      <div>
        <div class="p-5 mb-4 bg-light rounded-3">
          <div class="container-fluid py-5">
            <h1 class="display-5 fw-bold">MPC Controller </h1>
            <p class="col-md-8 fs-4">
              The team is composed of 4 students in Master of Engineering in
              Mechanical Engineering at UC Berkeley
            </p>
          </div>
        </div>
        <Container>
          <h2>Model Predictive Controller</h2>
          <div>
            <p>
              Model predictive control (MPC) is a control algorithm that allows
              for the precise control of dynamic systems. It is a type of
              feedback control algorithm, in which a model of the system is used
              to make predictions about the future behavior of the system. The
              control inputs are then computed based on the predictions, with
              the goal of achieving the desired system behavior. This is done
              iteratively, with the model being updated at each time step based
              on the measured system outputs.
            </p>
            <p>
              One of the key advantages of MPC is that it allows for the
              optimization of control inputs over a finite time horizon. This
              means that the control inputs can be chosen to not only achieve
              the desired system behavior, but also to minimize a cost function,
              which can encode various objectives such as energy consumption or
              control effort. Additionally, MPC algorithms can take into account
              constraints on the system, such as actuator limits or state
              constraints. In summary, MPC is a powerful control algorithm that
              allows for the precise and efficient control of dynamic systems,
              with the ability to optimize control inputs and incorporate
              constraints.
            </p>
          </div>
          <h2>Our implementation</h2>
          <div>
            <p>
              For our robot’s controller architecture our team decided to
              execute a Model Predictive Controller in order to guide the motion
              of our ball within its platform. Compared to traditional
              controller strategies, a Model Predictive Controller contains an
              internal understanding of how the real-world model will evolve
              over time provided inputs into the system. Using this information,
              the controller is able to evaluate multiple predicted states,
              compared to a single feedbacked state, and provide the best input
              in order to minimize the cost established by our target motion.
              Having this predictive feedforward capability allows Model
              Predictive Controllers to also take into consideration constraints
              on the state as well as constraints on resulting command input
              which came crucial to guiding smooth motion from our robot’s
              movement. If our MPC problem is formatted with the right
              simplifications, we are able to make this optimization problem
              convex, speeding up the time of calculation to fit within our 30
              Hz expectation.
            </p>
            <p>
              Because of the Sawyer Arm’s limited strength capabilities, our
              added platform weight meant that the low-level PID controller for
              robot movement would no longer be best suited for our application.
              In testing, requesting small step changes of pitch and roll lead
              to considerable accumulation of steady-state error, meaning our
              designed MPC would not be able to output solely joint-angle
              positions. Designing our own low-level controller with torque
              inputs was a possible word-around for this problem, but again in
              testing, recorded torque efforts provided by the Sawyer were not
              accurate enough for our purposes. This left one last option,
              controlling joint velocities. Diving into past projects and other
              course material, joint-velocity control seemed to provide enough
              power to overcome steady state disturbances in position, while
              retaining an accurate enough sensor reading for decent feedback
              information. Because of this, we would add an additional two
              parameters to our MPC design (pitch and roll), and have the
              resulting outputs of our MPC be pitch and roll velocity. In a
              similar fashion, we implemented constraints in our controller
              design in order to limit drastic changes in velocity commands over
              the resulting control sequence as we knew the robot would not be
              able to keep up effectively.{" "}
            </p>
            <p>
              The final model that is included in our MPC design is a Eulerian
              discretized model of general point-mass plate movement. Our model
              does not take into account rolling ball dynamics, friction, or
              other external disturbances. While including this information may
              make our MPC more accurate in its predictions, feedback provided
              by monitoring our system would be able to correct for any major
              prediction error. A simplified version of our model’s provided
              states, system dynamics, and resulting output command are shown
              below.
            </p>
          </div>
          <div>
            <img src={equation} alt="Equation" width="100%" height="auto" />
          </div>
          <div>
            <p>
              Simulating our MPC was a crucial tool in making sure our solve
              times, model accuracy, and resulting commands all stayed within
              the bounds or our expectations. Our MPC was tested against
              model-accurate and non-model accurate versions of a simulation
              environment. Even with cases such as 80% input, failing
              small-angle assumptions, or noisy position data, our MPC did a
              solid job at managing to reach desired positions and follow paths.
              A noisy simulation environment with reference tracking is shown
              below, with the red dot describing the desired position, green dot
              as the simulated position of the ball, and the blue markers
              indicating the MPC’s predicted future states.
            </p>
          </div>
          <div>
            <img src={simulaton} alt="loading..." width="100%" height="auto" />
          </div>
          <div>
            <p>
              For future improvement, positional steady state error seems to be
              the biggest problem as a result of our model’s assumption based
              inaccuracies. Because of plate friction and ball dynamics, there
              comes situations where the ball is quite close to the center, but
              our MPC requests angle changes that are not large enough to
              overcome the restrictions on motion. Integrating within our
              control loop a simple state estimator would likely be the best
              solution to overcome this model inaccuracy by introducing an
              understanding of the ongoing disturbances and state evolution
              within the system.
            </p>
          </div>
        </Container>
      </div>
    );
  }
}

export default Controller;
