import React from "react";
import {
  Scene, PerspectiveCamera, DirectionalLight, BufferGeometry, BufferAttribute, Points, WebGLRenderer,
  ShaderMaterial //PointsMaterial
} from 'three';
import * as OrbitControls from 'three-orbitcontrols';

import "./pointCloudView.scss";

const url = new URL(window.location.href);
const DELAY = Math.max(1, url.searchParams.get('delay') || 1); // Num frames to delay, minimum 1 for frame itself
const MAX_POINTS = url.searchParams.get('points') || 5000; // Max number of points per frame
const MAX_LENGTH = MAX_POINTS * DELAY * 3;
const POINT_SIZE = 0.01;

// Shader for colour based on height
const vertexShader = [
  'varying float min;',
  'varying float max;',
  'varying float height;',
  'void main() {',
  '    height = position.z;',
  '    gl_Position = projectionMatrix * modelViewMatrix * vec4( position, 1.0 );',
  '    if (height < min) {',
  '       min = height;',
  '   }',
  '}',
].join('\n');
const fragmentShader = [
  'varying float height;',
  'vec3 HUEtoRGB(float H) {',
  '    H = mod(H,1.0);',
  '    float R = abs(H * 6.0 - 3.0) - 1.0;',
  '    float G = 2.0 - abs(H * 6.0 - 2.0);',
  '    float B = 2.0 - abs(H * 6.0 - 4.0);',
  '    return clamp(vec3(R,G,B),0.0,1.0);',
  '}',
  'void main() {',
  '    gl_FragColor = vec4(HUEtoRGB(height / 2.0), 1.0 );',
  '}',
].join('\n');


class PointCloudView extends React.Component {
  constructor(props) {
    super(props);

    this.socket = null;
    this.animate = this.animate.bind(this);
    this.connect = this.connect.bind(this);
    this.connectToSocket = this.connectToSocket.bind(this);
  }

  componentDidMount() {
    this.scene = new Scene();
    this.camera = new PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.001, 200);
    this.camera.position.x = -1.5;
    this.camera.position.y = -3;
    this.camera.position.z = 1.5;
    this.camera.up.set(0, 0, 1);

    const dirLight = new DirectionalLight(0xdddddd);
    dirLight.position.set(0, 15, 15);
    this.scene.add(dirLight);

    this.geometry = new BufferGeometry();
    const positions = new Float32Array(MAX_LENGTH);
    this.geometry.addAttribute('position', new BufferAttribute(positions, 3));
    //const material = new PointsMaterial({size: POINT_SIZE}); // Plain colour
    const material = new ShaderMaterial({
      vertexShader: vertexShader,
      fragmentShader: fragmentShader
    });
    const particles = new Points(this.geometry, material);
    particles.frustumCulled = false;
    this.scene.add(particles);

    this.renderer = new WebGLRenderer();
    this.renderer.setSize(this.container.clientWidth, this.container.clientHeight);
    this.container.appendChild(this.renderer.domElement);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 1, 0);
    this.controls.maxPolarAngle = Math.PI / 2;

    if (this.props.lidarId) {
      this.connect();
    }
  }

  componentDidUpdate(prevProps, prevState) {
    if (prevProps.lidarId == this.props.lidarId) {
      return;
    }

    if (this.socket) {
      if (this.frameId) {
        cancelAnimationFrame(this.frameId);
      }
      this.socket.close();
    }

    // TODO: Reset geometry

    this.connect();
  }

  render() {
    return (
      <div className='point-cloud' ref={(el) => { this.container = el }}/>
    );
  }

  componentWillUnmount() {
    if (this.frameId) {
      cancelAnimationFrame(this.frameId);
    }
    this.container.removeChild(this.renderer.domElement);
  }

  connect() {
    this.connectToSocket();

    let index = 0; // Offset for rotating through stored frames
    let pointData = null;
    this.socket.onmessage = (e) => {
      // Always assign the latest buffer...
      // but only draw this new buffer at regular intervals.
      // That way message processing can happen as fast as
      // possible, and doesn't backup waiting on render.
      pointData = e.data;
    };

    // This does the heavy lifting.
    setInterval(() => {
      const buf = new Float32Array(pointData);
      // Reset index to 0 if the frame is too large
      if (index + buf.length >= MAX_LENGTH) {
        index = 0;
      }
      this.geometry.attributes.position.set(buf, index);
      index += buf.length;
      this.geometry.attributes.position.needsUpdate = true;
    }, 100);

    this.animate();
  }

  connectToSocket() {
    const host = window.document.location.host.replace(/:.*/, '');
    const port = process.env.PORT || 3002;
    this.socket = new WebSocket('ws://' + host + ':' + port + '/view/' + this.props.lidarId);
    this.socket.binaryType = 'arraybuffer';
    this.socket.onopen = (e) => {
      console.log('socket open', this.props.lidarId);
      // TODO: Clear out old points
    };
    this.socket.onerror = (e) => {
      console.log('socket errored', this.props.lidarId);
      // TODO: Try reconnecting to socket
    };
    this.socket.onclose = (e) => {
      console.log('socket closed', this.props.lidarId);
      // TODO: Try reconnecting to socket
    };
  }

  animate() {
    this.frameId = requestAnimationFrame(this.animate);
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }
}

export default PointCloudView;
