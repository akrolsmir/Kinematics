var $N = numeric;

var scene = new THREE.Scene();
var camera = new THREE.PerspectiveCamera(75, window.innerWidth/window.innerHeight, 0.1, 1000);

var renderer = new THREE.WebGLRenderer();
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

camera.position.z = 5;

var render = function () {
  requestAnimationFrame(render);

  // calculate endpoints
  arm[0].update();
  for (var i = 1; i < arm.length; i++) {
    arm[i].update(arm[i - 1]);
  };

  var dist = $N.sub(goal, arm[arm.length - 1].end.toArray());
  if($N.norm2(dist) > 0.1) {
    // update rots for next render loop
    var rotMats = [$N.identity(3)];
    var jcbMats = [arm[0].jacobianMatrix()];
    for(var i = 1; i < arm.length; i++) {
      rotMats.push($N.dot(arm[i].rotationMatrix(), rotMats[i - 1]));
      jcbMats.push($N.dot(rotMats[i], arm[i].jacobianMatrix()));
    }
    var j = concatMatrices(jcbMats);
    var jPlus = $N.dot($N.transpose(j), $N.inv($N.dot(j, $N.transpose(j))));
        
    var dr = $N.dot(jPlus, $N.mul(dist, 0.1));

    for(var i = 0; i < arm.length; i++) {
      arm[i].rot.add(new THREE.Vector3(dr[3 * i], dr[3 * i + 1], dr[3 * i + 2]));
    }
    alerted = false;
    }
  else if(!alerted) {
    alert("DONE!");
    alerted = true;
  }

  renderer.render(scene, camera);
};

var goal = [2, 1, 1];
var alerted = false;

function Joint(length, rot) {
  this.length = length;
  this.pos = new THREE.Vector3(0, 0, 0);
  this.end = new THREE.Vector3(0, 0, 0);
  this.rot = rot ? rot : new THREE.Vector3(0, 0, 0);

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

Joint.prototype.jacobianMatrix = function() {
  return crossMatrix([this.pos.x-this.end.x, this.pos.y-this.end.y, this.pos.z-this.end.z]);
}

Joint.prototype.rotationMatrix = function() {
  var theta = this.rot.length();
  var norm = this.rot.clone().normalize().toArray();
  var rx = crossMatrix(norm);
  return $N.add($N.mul(rx, Math.sin(theta)), 
    $N.identity(3), 
    $N.mul($N.dot(rx, rx), 1 - Math.cos(theta)));
}

function crossMatrix(v) {
  return [[0, -v[2], v[1]],
          [v[2], 0, -v[0]],
          [-v[2], v[1], 0]];
}

function concatMatrices(matrices) {
  return $N.transpose(Array.prototype.concat.apply([], matrices.map($N.transpose)));
}

var joint1 = new Joint(1, new THREE.Vector3(0, 0, 0));
var joint2 = new Joint(3, new THREE.Vector3(0, 0, 1.57));
var joint3 = new Joint(2, new THREE.Vector3(3, 0, 0));
var joint4 = new Joint(1, new THREE.Vector3(0.5, 0.2, 1.0));
var arm = [joint1, joint2, joint3, joint4];
render();