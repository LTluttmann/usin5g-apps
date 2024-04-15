import * as THREE from 'three';
import {OrbitControls} from 'three/examples/jsm/controls/OrbitControls.js'
import * as DAT from 'dat.gui';
import {GLTFLoader} from 'three/examples/jsm/loaders/GLTFLoader.js';
import {CSS2DRenderer, CSS2DObject} from 'three/examples/jsm/renderers/CSS2DRenderer.js'; 

import { getLatestMessage } from './mqttClient.js';

import stars from '../img/stars.jpg'; 
import tip from '../img/TIP.png'; 

// const ur10e = new URL('../assets/ur10e.gltf', import.meta.url);
const liebesschloss = new URL('../assets/Liebesschloss24_SetupV3.glb', import.meta.url);
const speedmarker = new URL('../assets/SpeedmarkMitAufnahme.glb', import.meta.url);
const mimaki = new URL('../assets/Mimaki7151plusII.glb', import.meta.url);
const ur10_gestell = new URL('../assets/UR10_Standalone.glb', import.meta.url);

// const ur10 = new URL('../assets/ur10.gltf', import.meta.url);
// const printer = new URL('../assets/printer.gltf', import.meta.url);

const renderer = new THREE.WebGLRenderer();

renderer.shadowMap.enabled = true; 
renderer.setSize(window.innerWidth, window.innerHeight);

document.body.appendChild(renderer.domElement);

const scene = new THREE.Scene();

const camera = new THREE.PerspectiveCamera(
    45,
    window.innerWidth / window.innerHeight,
    0.1,
    2000
);

const orbit = new OrbitControls(camera, renderer.domElement);

const axesHelper = new THREE.AxesHelper(5);
scene.add(axesHelper);

camera.position.set(100, 160, 100);
orbit.update(); 

// const boxGeometry = new THREE.BoxGeometry();
// const boxMaterial = new THREE.MeshBasicMaterial({color: 0x00FF00});
// const box = new THREE.Mesh(boxGeometry, boxMaterial);
// scene.add(box);

const planeGeometry = new THREE.PlaneGeometry(30, 30);
const planeMaterial = new THREE.MeshStandardMaterial({
    color: 0xFFF6F6,
    side: THREE.DoubleSide
});
const plane = new THREE.Mesh(planeGeometry, planeMaterial);
scene.add(plane);
plane.rotation.x = -0.5 * Math.PI;
plane.receiveShadow = true; 

const gridHelper = new THREE.GridHelper(30);
scene.add(gridHelper);

const sphereGeometry = new THREE.SphereGeometry(5, 50, 50);
const sphereMaterial = new THREE.MeshStandardMaterial({
    color: 0x0000FF,
    wireframe: false
});
const sphere = new THREE.Mesh(sphereGeometry,sphereMaterial);
scene.add(sphere);
sphere.position.set(-500,-500,-500); 
sphere.castShadow = true;

const ambientLight = new THREE.AmbientLight(0x404040);
scene.add(ambientLight);

//const directionalLight = new THREE.DirectionalLight(0xFFFFFF, 0.8);
//scene.add(directionalLight);
//directionalLight.position.set(-10, 10, 10);
//directionalLight.castShadow = true; 
//directionalLight.shadow.camera.left = -12;
//directionalLight.shadow.camera.bottom = -10;
//directionalLight.shadow.camera.top = 10; 

//const dLightHelper = new THREE.DirectionalLightHelper(directionalLight, 5);
//scene.add(dLightHelper);

//const dLightShadowHelper = new THREE.CameraHelper(directionalLight.shadow.camera); 
//scene.add(dLightShadowHelper); 

const spotLight = new THREE.SpotLight(0xffffff, 10000); 
scene.add(spotLight);
spotLight.position.set(400, 200, 200); 
spotLight.castShadow = true;


const spotLightHelper = new THREE.SpotLightHelper(spotLight); 
scene.add(spotLightHelper);

//scene.fog = new THREE.Fog(0xffffff, 0, 200);
scene.fog = new THREE.FogExp2(0xffffff, 0.001); 

//renderer.setClearColor(0xFFEA00);

const textureLoader = new THREE.TextureLoader();
//scene.background = textureLoader.load(stars);
renderer.setClearColor(0x939388);
const cubeTextureLoader = new THREE.CubeTextureLoader(); 
// scene.background = cubeTextureLoader.load([
//     stars,
//     stars,
//     stars,
//     stars,
//     stars,
//     stars
// ]);

const box2Geometry = new THREE.BoxGeometry(5, 5, 5);
const box2Material = new THREE.MeshBasicMaterial({
    //color: 0x00FF00,
    //map: textureLoader.load(tip)
});
const box2MultiMaterial = [
    new THREE.MeshBasicMaterial({color: 0x939388}),
    new THREE.MeshBasicMaterial({map: textureLoader.load(stars)}),
    new THREE.MeshBasicMaterial({map: textureLoader.load(tip)}),
    new THREE.MeshBasicMaterial({map: textureLoader.load(stars)}),
    new THREE.MeshBasicMaterial({map: textureLoader.load(stars)}),
    new THREE.MeshBasicMaterial({map: textureLoader.load(tip)})
];
const box2 = new THREE.Mesh(box2Geometry, box2MultiMaterial);
scene.add(box2); 
box2.position.set(-505,-505,-505);
//box2.material.map = textureLoader.load(tip);

const plane2Geometry = new THREE.PlaneGeometry(10, 10, 10, 10); 
const plane2Material = new THREE.MeshBasicMaterial({
    color: 0x980bbe,
    wireframe: true,
    side: THREE.DoubleSide
}); 

const plane2 = new THREE.Mesh(plane2Geometry, plane2Material);
scene.add(plane2); 
plane2.position.set(-500,-500,-500);


const sphere2Geometry = new THREE.SphereGeometry(40);
const sphere2Material = new THREE.MeshBasicMaterial({color: 0xFFFFFF});
/* const vShader = `
    void main() {
        gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
    }
`;

const fShader = `
    void main() {
        gl_FragColor = vec4(0.5, 0.5, 1.0, 1.0);
    }
`; */
const sphere2Shader = new THREE.ShaderMaterial({
    vertexShader: document.getElementById('vertexShader').textContent,
    fragmentShader: document.getElementById('fragmentShader').textContent
});

const sphere2 = new THREE.Mesh(sphere2Geometry, sphere2Shader); 
scene.add(sphere2);
sphere2.position.set(-500,-500,-500);


const gltfLoader = new GLTFLoader();


// gltfLoader.load(untergestell.href, function(gltf) {
//     const model_gestell = gltf.scene;
//     scene.add(model_gestell);
//     model_gestell.rotation.x = -1/2 * Math.PI;

// }, undefined, function(error) {
//     console.error(error);
// });

// gltfLoader.load(ur10e.href, function(gltf) {
//     const model_ur10e = gltf.scene;
//     scene.add(model_ur10e);
//     model_ur10e.position.set(0, 84, -5);
// }, undefined, function(error) {
//     console.error(error);
// });
gltfLoader.load(liebesschloss.href, function(gltf) {
    gltf.scene.scale.set(35*gltf.scene.scale.x, 35*gltf.scene.scale.y, 35 * gltf.scene.scale.z);
    const model_liebesschloss = gltf.scene;
    scene.add(model_liebesschloss);
    model_liebesschloss.position.set(-150, 24.5, 0);
}, undefined, function(error) {
    console.error(error);
});

gltfLoader.load(ur10_gestell.href, function(gltf) {
    gltf.scene.scale.set(35*gltf.scene.scale.x, 35*gltf.scene.scale.y, 35 * gltf.scene.scale.z);
    const model_ur10_gestell = gltf.scene;
    scene.add(model_ur10_gestell);
    model_ur10_gestell.position.set(-122, 24.5, -2);
}, undefined, function(error) {
    console.error(error);
});

gltfLoader.load(speedmarker.href, function(gltf) {
    gltf.scene.scale.set(35*gltf.scene.scale.x, 35*gltf.scene.scale.y, 35 * gltf.scene.scale.z);
    const model_speedmarker = gltf.scene;
    scene.add(model_speedmarker);
    model_speedmarker.position.set(20, 5, -10);
    model_speedmarker.rotation.x = 0.5 * Math.PI;
}, undefined, function(error) {
    console.error(error);
});

gltfLoader.load(mimaki.href, function(gltf) {
    gltf.scene.scale.set(35*gltf.scene.scale.x, 35*gltf.scene.scale.y, 35 * gltf.scene.scale.z);
    const model_mimaki = gltf.scene;
    scene.add(model_mimaki);
    model_mimaki.position.set(18, 19, -100);
    model_mimaki.rotation.y = 0.5 * Math.PI;
}, undefined, function(error) {
    console.error(error);
});


const gui = new DAT.GUI();

const options = {
    sphereColor: '#ffea00',
    boxColor: '#ffea00',
    wireframe: false,
    speed: 0.01,
    angle: 0.45,
    penumbra: 0,
    intensity: 1200,
    decay: 1
};

gui.addColor(options, 'sphereColor').onChange(function(e){
    sphere.material.color.set(e)
});

gui.add(options, 'wireframe').onChange(function(e){
    sphere.material.wireframe = e
});

// gui.addColor(options, 'boxColor').onChange(function(e){
//     box.material.color.set(e)
// });

let step = 0;

const mousePosition = new THREE.Vector2();
window.addEventListener('mousemove', function(e) {
    mousePosition.x = (e.clientX / window.innerWidth) * 2 - 1;
    mousePosition.y = - (e.clientY / window.innerHeight) * 2 + 1; 
});

 

gui.add(options, 'speed', 0, 0.1);
gui.add(options, 'angle', 0, 1);
gui.add(options, 'penumbra', 0, 1);
gui.add(options, 'intensity', 0, 20000);
gui.add(options, 'decay', 0, 1);

const sphereID = sphere.id;

box2.name = 'theBox2';


const labelRenderer = new CSS2DRenderer();
labelRenderer.setSize(window.innerWidth, window.innerHeight); 
labelRenderer.domElement.style.position = 'absolute';
labelRenderer.domElement.style.top = '0px'; 
labelRenderer.domElement.style.pointerEvents = 'none';
document.body.appendChild(labelRenderer.domElement);

// const pLabel = new CSS2DObject(p); 
// scene.add(pLabel); 
// pLabel.position.set(10, 10, 10); 

function createSensorPoint(name, x, y, z) {
    const kugel = new THREE.SphereGeometry(1.5);
    const mat = new THREE.MeshBasicMaterial({color: 0xFF0000});
    const mesh = new THREE.Mesh(kugel, mat);
    mesh.position.set(x, y, z); 
    mesh.name = name; 
    return mesh;
}; 

const group = new THREE.Group();

const sphereMesh1 = createSensorPoint('UR10 data', -110, 32, -1);
group.add(sphereMesh1);
const sphereMesh2 = createSensorPoint('Printer data', 0, 55, 0);
group.add(sphereMesh2);
scene.add(group);

const p = document.createElement('p');
p.className = 'tooltip'; 
const pContainer = document.createElement('div');
pContainer.appendChild(p);
const cPointLabel = new CSS2DObject(pContainer); 
scene.add(cPointLabel); 

const mousePos = new THREE.Vector2();
const rayCaster = new THREE.Raycaster();

window.addEventListener('mousemove', function(e) {
    mousePos.x = (e.clientX / window.innerWidth) * 2 - 1; 
    mousePos.y = -(e.clientY / window.innerHeight) * 2 + 1; 

    rayCaster.setFromCamera(mousePos, camera); 
    const intersects = rayCaster.intersectObject(group);
    if(intersects.length > 0) {
        switch (intersects[0].object.name) {
            case 'UR10 data':
                p.className = 'tooltip show';
                cPointLabel.position.set(-110, 32, -1);
                p.textContent = `Data UR10 shown here: ${getLatestMessage()}`
                break;
            
            case 'Printer data':
                p.className = 'tooltip show';
                cPointLabel.position.set(0, 55, 0);
                p.textContent = 'Printer data shown here'
                break;
        
            default:
                break;
        }
    } else {
        p.className ='tooltip hide';
    }
});


function animate (time) {
    // box.rotation.x =  time / 1000;
    // box.rotation.z =  time / 1000;

    step += options.speed;
    sphere.position.y = 10 * Math.abs(Math.sin(step));
    
    spotLight.angle = options.angle;
    spotLight.penumbra = options.penumbra;
    spotLight.intensity = options.intensity;
    spotLight.decay = options.decay;

    rayCaster.setFromCamera(mousePosition, camera);
    const intersects = rayCaster.intersectObjects(scene.children); 
    console.log(intersects); 

    for (let i = 0; i < intersects.length; i++) {
        if(intersects[i].object.id === sphereID)
            intersects[i].object.material.color.set(0xFF0000);

        if(intersects[i].object.name === 'theBox2') {
            intersects[i].object.rotation.x = time / 1000; 
            intersects[i].object.rotation.y = time / 1000;
        }
    }

    plane2.geometry.attributes.position.array[0] = 10 * Math.random();
    plane2.geometry.attributes.position.array[1] = 10 * Math.random();
    plane2.geometry.attributes.position.array[2] = 10 * Math.random();
    const lastPointZ = plane2.geometry.attributes.position.array.length -1;
    plane2.geometry.attributes.position.array[lastPointZ] = 10 * Math.random();
    plane2.geometry.attributes.position.needsUpdate = true;
    labelRenderer.render(scene, camera); 
    renderer.render(scene, camera);
}

window.addEventListener('resize', function() {
    camera.aspect = window.innerWidth / window.innerHeight; 
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
    labelRenderer.setSize(this.window.innerWidth, this.window.innerHeight);  
});

renderer.setAnimationLoop(animate);






