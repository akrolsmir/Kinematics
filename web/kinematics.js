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

  var dist = $N.sub(goal, arm[arm.length - 1].end);
  if($N.norm2(dist) > 0.1) {
    // update rots for next render loop
    var rotMats = [$N.identity(3)];
    var jcbMats = [arm[0].jacobianMatrix()];
    for(var i = 1; i < arm.length; i++) {
      rotMats.push($N.dot(arm[i].rotationMatrix(), rotMats[i - 1]));
      jcbMats.push($N.dot(rotMats[i], arm[i].jacobianMatrix()));
    }
    var j = $N.concat(jcbMats);
    var jPlus = $N.dot($N.transpose(j), $N.inv($N.dot(j, $N.transpose(j))));
        
    var dr = $N.dot(jPlus, $N.mul(dist, 0.1));

    for(var i = 0; i < arm.length; i++) {
      $N.addeq(arm[i].rot, dr.slice(3 * i, 3 * (i + 1)));
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
  this.pos = [0, 0, 0];
  this.end = [0, 0, 0];
  this.rot = rot ? rot : [0, 0, 0];

  // Initializes this joint in the scene
  var material = new THREE.LineBasicMaterial({
    color: 0x0000ff
  });

  var geometry = new THREE.Geometry();
  geometry.vertices.push(new THREE.Vector3().fromArray(this.pos));
  geometry.vertices.push(new THREE.Vector3().fromArray(this.end));

  var line = new THREE.Line( geometry, material );
  scene.add( line );

  this.geometry = geometry;
}

Joint.prototype.update = function(prev) {
  // Recalculate the pos and end based on prev and this.rot
  this.pos = prev ? prev.end : this.pos;
  var x = prev ? $N.mul($N.normal($N.sub(prev.end, prev.pos)), this.length) : [this.length, 0, 0];
  var theta = $N.norm2(this.rot);
  if (theta == 0) {
    this.end = x;
  } else {
    var norm = $N.normal(this.rot);
    this.end = $N.add($N.mul(norm, $N.dot(norm, x)),
      $N.mul($N.cross(norm, x), Math.sin(theta)),
      $N.mul($N.cross(norm, $N.cross(norm, x)), -Math.cos(theta)));
  }
  $N.addeq(this.end, this.pos);

  // Update the geometry
  this.geometry.vertices[0] = new THREE.Vector3().fromArray(this.pos);
  this.geometry.vertices[1] = new THREE.Vector3().fromArray(this.end);
  this.geometry.verticesNeedUpdate = true;
}

Joint.prototype.jacobianMatrix = function() {
  return crossMatrix($N.sub(this.pos, this.end));
}

Joint.prototype.rotationMatrix = function() {
  var theta = $N.norm2(this.rot);
  var rx = crossMatrix($N.normal(this.rot));
  return $N.add($N.mul(rx, Math.sin(theta)), 
    $N.identity(3), 
    $N.mul($N.dot(rx, rx), 1 - Math.cos(theta)));
}

function crossMatrix(v) {
  return [[0, -v[2], v[1]],
          [v[2], 0, -v[0]],
          [-v[2], v[1], 0]];
}

var joint1 = new Joint(1, [0, 0, 0]);
var joint2 = new Joint(3, [0, 0, 1.57]);
var joint3 = new Joint(2, [3, 0, 0]);
var joint4 = new Joint(1, [0.5, 0.2, 1.0]);
var arm = [joint1, joint2, joint3, joint4];
render();