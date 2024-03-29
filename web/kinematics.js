var scene = new THREE.Scene();
var camera = new THREE.PerspectiveCamera(75, window.innerWidth/window.innerHeight, 0.1, 1000);

var renderer = new THREE.WebGLRenderer({canvas: canvas1});
renderer.setSize(window.innerWidth / 2, window.innerHeight / 2);
// document.body.appendChild(renderer.domElement);

camera.position.z = 5;

var render = function () {
  requestAnimationFrame(render);

  moveGoal();
  // Display the goal
  oct.position = new THREE.Vector3().fromArray(goal);
  frame.position = oct.position;
  oct.rotation.y += 0.1;
  frame.rotation.y += 0.1;

  if(stick)
    zoom();
  else
    approach(arm, goal);

  for(var i = 0; i < arm.length; i++) {
    arm[i].show();
  }

  renderer.render(scene, camera);
};

var stick = true;
function zoom() {
  for(var i = 0; i < 100 && approach(arm, goal) > 0.0001; i++);  
}

// Moves the arm closer to goal. Returns the length of dr.
function approach(arm, goal) {
  // Calculate endpoints
  arm[0].update();
  for (var i = 1; i < arm.length; i++) {
    arm[i].update(arm[i - 1]);
  };

  var dr = [0];
  var dist = $N.sub(goal, arm[arm.length - 1].end);
  if($N.norm2(dist) > 0.01) {
    // update rots for next render loop
    var rotMats = [$N.identity(3)];
    var jcbMats = [arm[0].jacobianMatrix(rotMats[0], arm[arm.length - 1].end)];
    for(var i = 1; i < arm.length; i++) {
      rotMats.push($N.dot(rotMats[i - 1], arm[i].rotationMatrix()));
      jcbMats.push($N.dot(rotMats[i], arm[i].jacobianMatrix(rotMats[i], arm[arm.length - 1].end)));
    }
    var j = $N.concat(jcbMats);
    // var jPlus = $N.dot($N.transpose(j), $N.inv($N.dot(j, $N.transpose(j))));
    // damp = 1 / (0.1 + $N.norm2(dist)) + 1;

    // Damped Least Squares
    var jPlus = $N.dot($N.transpose(j), $N.inv(
      $N.add($N.dot(j, $N.transpose(j)), $N.mul($N.identity(3), damp))));

    // ClampMag:
    // dist = $N.mul($N.normal(dist), Math.max($N.norm2(dist), 0.5));
    dr = $N.dot(jPlus, dist);

    for(var i = 0; i < arm.length; i++) {
      $N.addeq(arm[i].rot, dr.slice(3 * i, 3 * (i + 1)));
    }
  }
  return $N.norm2(dr);
}
var damp = 20;

// Set up the goal point
var geometry = new THREE.OctahedronGeometry(.1);
var oct = new THREE.Mesh( geometry, new THREE.MeshBasicMaterial({color: 0xffff00}));
var frame = new THREE.Mesh( geometry, new THREE.MeshBasicMaterial({color: 0x000000, wireframe: true}));
scene.add( oct );
scene.add( frame );

var angle = 0;
var goal = [2, 1, 1];

function moveGoal() {
  angle += 0.010;
  goal = [8 * Math.sin(0.5 * angle), 4 * Math.sin(angle), 0];
}

function resetArm() {
  for(var i = 0; i < arm.length; i++) {
    scene.remove(arm[i].line);
  }
}

// Set up the arm and render everything
var joint1 = new Joint(1, [0, 0, 0]);
var joint2 = new Joint(3, [0, 0, 1.57]);
var joint3 = new Joint(2, [3, 0, 0]);
var joint4 = new Joint(1, [0.5, 0.2, 1.0]);
var arm = [joint1, joint2, joint3, joint4,];
render();
