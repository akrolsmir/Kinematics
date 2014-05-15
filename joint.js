function Joint(length, rot) {
  this.length = length;
  this.pos = [0, 0, 0];
  this.end = [0, 0, 0];
  this.rot = rot ? rot : [0, 0, 0];

  // Initializes this joint in the scene
  var material = new THREE.LineBasicMaterial({
    color: 0x0000ff
  });

  this.geometry = new THREE.Geometry();
  this.geometry.vertices.push(new THREE.Vector3().fromArray(this.pos));
  this.geometry.vertices.push(new THREE.Vector3().fromArray(this.end));

  this.line = new THREE.Line( this.geometry, material );
  scene.add( this.line );
}

Joint.prototype.update = function(prev) {
  // Recalculate the pos and end based on prev and this.rot
  this.pos = prev ? prev.end : this.pos;
  var x = prev ? $N.mul($N.normal($N.sub(prev.end, prev.pos)), this.length) : [this.length, 0, 0];
  this.end = $N.add(this.pos, $N.dot(this.rotationMatrix(), x));
}

Joint.prototype.show = function() {
  // Update the geometry
  this.geometry.vertices[0] = new THREE.Vector3().fromArray(this.pos);
  this.geometry.vertices[1] = new THREE.Vector3().fromArray(this.end);
  this.geometry.verticesNeedUpdate = true;
}

Joint.prototype.jacobianMatrix = function(trans, end) {
  return crossMatrix($N.dot($N.transpose(trans), $N.sub(this.pos, this.end)));
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
          [-v[1], v[0], 0]];
}