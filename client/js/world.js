import * as THREE from 'three';
import WebGL from 'three/addons/capabilities/WebGL.js';
import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const loader = new OBJLoader();

const scene = new THREE.Scene();
scene.background = new THREE.Color(0xebeae8);

const camera = new THREE.PerspectiveCamera( 75, (window.innerWidth / 2.0) / window.innerHeight, 0.1, 1000 );

const world_host = document.getElementById("world-host");
const renderer = new THREE.WebGLRenderer();
renderer.setSize(window.innerWidth / 2.0, window.innerHeight);
world_host.appendChild(renderer.domElement)

const controls = new OrbitControls( camera, renderer.domElement );
controls.enableDamping = true;

const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
scene.add( directionalLight );

const ambient = new THREE.AmbientLight(0xffffff, 1.0);
scene.add(ambient);

camera.position.y = 5;
camera.position.z = -5;
camera.lookAt(scene.position);
controls.update();

const car_mesh = new THREE.MeshStandardMaterial({ color: 0x696969 })
loader.load( 'models/suv.obj', function(object) {
    object.traverse((mesh) => {
        mesh.material = car_mesh;
    });

    object.scale.set(0.7, 0.7, 0.7);

    object.position.z = -3;
    object.position.y = -0.5;

	scene.add(object);
}, undefined, function (error) {
	console.error( error );
});

const stop_sign_mesh = new THREE.MeshStandardMaterial({ color: 0xff0000 })
loader.load( 'models/stop_sign.obj', function(object) {
    object.traverse((mesh) => {
        mesh.material = stop_sign_mesh;
    });

    object.scale.set(0.3, 0.3, 0.3);

    object.position.z = 2;
    object.position.x = -1;

    object.rotation.y = 3.14 / 2.0;

	scene.add(object);
}, undefined, function (error) {
	console.error( error );
});

function animate() {
	requestAnimationFrame(animate);

    controls.update();

	renderer.render(scene, camera);
}

if (WebGL.isWebGLAvailable()) {
	// Initiate function or other initializations here
	animate();
} else {
	const warning = WebGL.getWebGLErrorMessage();
	world_host.replaceChild(warning, renderer.domElement);
}