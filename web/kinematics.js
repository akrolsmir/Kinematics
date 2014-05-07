//var $N = numeric;

var scene = new THREE.Scene();
var camera = new THREE.PerspectiveCamera(75, window.innerWidth/window.innerHeight, 0.1, 1000);

var renderer = new THREE.WebGLRenderer();
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

var geometry = new THREE.CubeGeometry(1,1,1);
var material = new THREE.MeshBasicMaterial({color: 0x00ff00});
var cube = new THREE.Mesh(geometry, material);
// scene.add(cube);

camera.position.z = 5;

var render = function () {
  requestAnimationFrame(render);

  joint.update();
  joint2.update(joint);
  // joint.rot.add(new THREE.Vector3(0.01, 0.01, 0.01));
  joint.rot = new THREE.Vector3(0, 0, 0);
  joint2.rot = new THREE.Vector3(0, 0, 1.57);
  // console.log(joint.end);

  renderer.render(scene, camera);
};

function Joint(length) {
  this.length = length;
  this.pos = new THREE.Vector3(0, 0, 0);
  this.end = new THREE.Vector3(0, 0, 0);
  this.rot = new THREE.Vector3(0, 0, 0);

  // Initializes this joint in the scene
  var material = new THREE.LineBasicMaterial({
    color: 0x0000ff
  });

  var geometry = new THREE.Geometry();
  geometry.vertices.push( this.pos );
  geometry.vertices.push( this.end );

  var line = new THREE.Line( geometry, material );
  scene.add( line );

  this.geometry = geometry;
}

Joint.prototype.update = function(prev) {
  // Recalculate the pos and end based on prev and this.rot
  this.pos = prev ? prev.end : this.pos;
  var x = prev ? prev.end.clone().sub(prev.pos).setLength(this.length) : new THREE.Vector3(this.length, 0, 0);
  var theta = this.rot.length();
  if (theta == 0) {
    this.end = x;
  } else {
    var norm = this.rot.clone().normalize();
    this.end = norm.clone().multiplyScalar(norm.dot(x))
                .add(norm.clone().cross(x).multiplyScalar(Math.sin(theta)))
                .sub(norm.clone().cross(norm.clone().cross(x)).multiplyScalar(Math.cos(theta)));
  }
  this.end.add(this.pos);

  // Update the geometry
  this.geometry.vertices[0] = this.pos;
  this.geometry.vertices[1] = this.end;
  this.geometry.verticesNeedUpdate = true;
}

var joint = new Joint(1);
var joint2 = new Joint(3);
render();