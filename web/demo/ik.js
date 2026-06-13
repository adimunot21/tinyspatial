// ik.js — drag-to-reach inverse kinematics, powered by tinyspatial (WASM).
//
// Click or drag anywhere in the box: the real C++ position-only damped
// least-squares solver (Robot.solveIkPosition) finds joint angles that put the
// tool tip on your cursor, warm-started from the current pose. The arm and the
// target marker are redrawn from the solver output. Same 2-link planar arm as
// the FK demo, with a fixed "tool" frame at the tip.
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

function toScreen(x, y) {
  return { x: ORIGIN.x + x * SCALE, y: ORIGIN.y - y * SCALE };
}
function toWorld(sx, sy) {
  return { x: (sx - ORIGIN.x) / SCALE, y: (ORIGIN.y - sy) / SCALE };
}

function main(Module) {
  const robot = new Module.Robot(URDF);
  const toolLink = robot.njoints() - 1;
  let q = new Array(robot.nq()).fill(0.3);
  let target = { x: 1.0, y: 0.8 };

  const svg = document.getElementById('arm');
  const polyline = document.createElementNS(SVG_NS, 'polyline');
  polyline.setAttribute('class', 'arm-link');
  svg.appendChild(polyline);
  const dots = [];
  const targetDot = document.createElementNS(SVG_NS, 'circle');
  targetDot.setAttribute('r', '9');
  targetDot.setAttribute('class', 'target-dot');
  svg.appendChild(targetDot);

  function renderArm() {
    const flat = robot.jointPositions(q);
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
    const ts = toScreen(target.x, target.y);
    targetDot.setAttribute('cx', ts.x);
    targetDot.setAttribute('cy', ts.y);
  }

  function solveTo(wx, wy) {
    target = { x: wx, y: wy };
    const res = robot.solveIkPosition(toolLink, wx, wy, 0.0, q);
    const out = [];
    for (let i = 0; i < res.q.length; i++) {
      out.push(res.q[i]);
    }
    q = out;
    document.getElementById('readout').textContent =
      'target (' +
      wx.toFixed(2) +
      ', ' +
      wy.toFixed(2) +
      ') — ' +
      (res.converged ? 'reached' : 'out of reach') +
      ' in ' +
      res.iterations +
      ' iterations';
    renderArm();
  }

  let dragging = false;
  function handle(evt) {
    const rect = svg.getBoundingClientRect();
    const w = toWorld(evt.clientX - rect.left, evt.clientY - rect.top);
    solveTo(w.x, w.y);
  }
  svg.addEventListener('mousedown', function (e) {
    dragging = true;
    handle(e);
  });
  svg.addEventListener('mousemove', function (e) {
    if (dragging) {
      handle(e);
    }
  });
  window.addEventListener('mouseup', function () {
    dragging = false;
  });

  solveTo(target.x, target.y);
}

createTinyspatial()
  .then(main)
  .catch(function (e) {
    document.getElementById('readout').textContent = 'Failed to load WASM module: ' + e;
  });
