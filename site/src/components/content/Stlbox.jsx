import React, { Component } from "react";
import { StlViewer } from "react-stl-viewer";
import stlFile from "../../assets/stl/assembly.stl";

const style = {
  top: 0,
  left: 0,
  width: "500px",
  height: "30vw",
};

const ModelProps = {
  scale: 1,
};

class Stlbox extends Component {
  state = {};
  render() {
    return (
      <div>
        <StlViewer
          style={style}
          ModelProps={ModelProps}
          orbitControls
          showAxes
          url={stlFile}
        />
      </div>
    );
  }
}

export default Stlbox;
