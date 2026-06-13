// app.js — interactive forward-kinematics demo powered by tinyspatial compiled
// to WebAssembly. Drag the sliders; the arm is drawn entirely from the joint
// positions the real C++ `forward_kinematics` returns through embind.
//
// The robot is a 2-link planar arm (two revolute joints about z) plus a fixed
// "tool" frame at the tip, so FK returns three points: base, elbow, tool — a
// two-segment polyline. Same URDF dialect as the test fixtures.
'use strict';

const URDF = `<?xml version="1.0"?>
<robot name="two_link">
  <link name="base_link"/>
  <link name="link1">
    <inertial><origin xyz="0.5 0 0" rpy="0 0 0"/><mass value="1.0"/>
      <inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial>
  </link>
  <link name="link2">
    <inertial><origin xyz="0.5 0 0" rpy="0 0 0"/><mass value="1.0"/>
      <inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial>
  </link>
  <link name="tool"/>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/><child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="2.0"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="link1"/><child link="link2"/>
    <origin xyz="1.0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="2.0"/>
  </joint>
  <joint name="tool" type="fixed">
    <parent link="link2"/><child link="tool"/>
    <origin xyz="1.0 0 0" rpy="0 0 0"/>
  </joint>
</robot>`;

const SVG_NS = 'http://www.w3.org/2000/svg';
const SCALE = 90; // pixels per metre
const ORIGIN = { x: 220, y: 220 };

// World frame: x right, y up. SVG y points down, so flip it.
function toScreen(x, y) {
  return { x: ORIGIN.x + x * SCALE, y: ORIGIN.y - y * SCALE };
}

function main(Module) {
  const robot = new Module.Robot(URDF);
  const nq = robot.nq();
  const q = new Array(nq).fill(0.0);

  const svg = document.getElementById('arm');
  const polyline = document.createElementNS(SVG_NS, 'polyline');
  polyline.setAttribute('class', 'arm-link');
  svg.appendChild(polyline);
  const dots = [];

  const controls = document.getElementById('controls');
  for (let k = 0; k < nq; k++) {
    const label = document.createElement('label');
    label.textContent = 'joint ' + (k + 1);
    const slider = document.createElement('input');
    slider.type = 'range';
    slider.min = '-3.14159';
    slider.max = '3.14159';
    slider.step = '0.01';
    slider.value = '0';
    slider.addEventListener('input', function () {
      q[k] = parseFloat(slider.value);
      render();
    });
    label.appendChild(slider);
    controls.appendChild(label);
  }

  function render() {
    const flat = robot.jointPositions(q); // [x0,y0,z0, x1,y1,z1, ...]
    const n = flat.length / 3;
    const points = [];
    for (let i = 0; i < n; i++) {
      const p = toScreen(flat[3 * i], flat[3 * i + 1]);
      points.push(p.x + ',' + p.y);
    }
    polyline.setAttribute('points', points.join(' '));

    while (dots.length < n) {
      const c = document.createElementNS(SVG_NS, 'circle');
      c.setAttribute('r', '7');
      c.setAttribute('class', 'joint-dot');
      svg.appendChild(c);
      dots.push(c);
    }
    for (let i = 0; i < n; i++) {
      const p = toScreen(flat[3 * i], flat[3 * i + 1]);
      dots[i].setAttribute('cx', p.x);
      dots[i].setAttribute('cy', p.y);
    }

    const tipX = flat[3 * (n - 1)];
    const tipY = flat[3 * (n - 1) + 1];
    document.getElementById('readout').textContent =
      'end-effector: (' + tipX.toFixed(3) + ', ' + tipY.toFixed(3) + ') m';
  }

  render();
}

createTinyspatial()
  .then(main)
  .catch(function (e) {
    document.getElementById('readout').textContent = 'Failed to load WASM module: ' + e;
  });
