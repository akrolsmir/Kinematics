var node = document.getElementsByTagName("head")[0] || document.body;
var script = document.createElement("script");
node.appendChild(script);

function inject() {
  node.removeChild(script);
  script = document.createElement("script");
  script.type = "text/javascript";
  var textnode = document.createTextNode(document.getElementById("text").value);
  script.appendChild(textnode);
  node.appendChild(script);
}