import {
  Scene, PerspectiveCamera, DirectionalLight, BufferGeometry, BufferAttribute, Points, WebGLRenderer,
  ShaderMaterial //PointsMaterial
} from 'three';
import * as OrbitControls from 'three-orbitcontrols';

const DELAY = 35; // Num frames to delay
const MAX_POINTS = 5000 * DELAY;
const MAX_LENGTH = MAX_POINTS * 3;
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

const scene = new Scene();
const camera = new PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.001, 200);
camera.position.x = -1.5;
camera.position.y = -3;
camera.position.z = 1.5;
camera.up.set(0, 0, 1);

const dirLight = new DirectionalLight(0xdddddd);
dirLight.position.set(0, 15, 15);
scene.add(dirLight);

const geometry = new BufferGeometry();
const positions = new Float32Array(MAX_LENGTH);
geometry.addAttribute('position', new BufferAttribute(positions, 3));
//const material = new PointsMaterial({size: POINT_SIZE}); // Plain colour
const material = new ShaderMaterial({
    vertexShader: vertexShader,
    fragmentShader: fragmentShader
});
const particles = new Points(geometry, material);
particles.frustumCulled = false;
scene.add(particles);

const renderer = new WebGLRenderer();
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 1, 0);
controls.maxPolarAngle = Math.PI / 2;

const host = window.document.location.host.replace(/:.*/, '');
const port = process.env.PORT || 3002;
let pointCloudSocket;
function connect() {
    pointCloudSocket = new WebSocket('ws://'+ host + ':' + port);
    pointCloudSocket.binaryType = 'arraybuffer';
    pointCloudSocket.onopen = (e) => {
        console.log('socket open');
    };
    pointCloudSocket.onclose = (e) => {
        console.log('socket closed');
    };
};
connect();

function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
}
animate();

// Offset for rotating through stored frames
let index = 0;
pointCloudSocket.onmessage = (e) => {
    const buf = new Float32Array(e.data);
    geometry.attributes.position.set(buf, index);
    index += buf.length;
    if (index >= MAX_LENGTH) {
        index = 0;
    }
    geometry.attributes.position.needsUpdate = true;
};
