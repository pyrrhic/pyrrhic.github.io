var ns = {};

ns.mapLoader = function() {
	return {
		getLayerByName: function(map, layerName) {
	        var objectLayers = map.objects;
	        return (objectLayers.hasOwnProperty(layerName)) ? objectLayers[layerName] : null;
	    },

	    getRectangleObjectAsPhaserRectangle: function(layer) {
	        var rectangles = [];
	        for (var i = 0; i < layer.length; i++) {
	            var object = layer[i];
	            var rect = new Phaser.Rectangle(object.x, object.y, object.width, object.height);

	            rectangles.push(rect);
	        }

	        return rectangles;
	    },

	    getPolygonObjectsAsLines: function(layer) {
	        var lines = [];
	        for (var i = 0; i < layer.length; i++) {
	            var object = layer[i];

	            var points = [];
	            for (var poly = 0; poly < object.polygon.length; poly++) {
	                points[poly] = [];

	                var x = object.polygon[poly][0] + object.x;
	                points[poly].push(x);

	                var y = object.polygon[poly][1] + object.y;
	                points[poly].push(y);
	            }

	            //when we have 2 points, draw a line. so get 1 pt, wait, get 2nd point, draw from 2nd to 1st. etc.
	            for (var p = 0; p < points.length; p++) {
	                if (p > 0) {
	                    var line = new Phaser.Line(points[p][0], points[p][1], points[p-1][0], points[p-1][1]);
	                    lines.push(line);
	                }
	                if ((p + 1) == points.length) { //if last point, draw line from last to first point
	                    var line = new Phaser.Line(points[p][0], points[p][1], points[0][0], points[0][1]);
	                    lines.push(line);
	                }
	            }
	        }

	        return lines;
	    }		
	};
} ();

//path finder package ported from my Java implementation
ns.Vector2 = function(x, y) {
	this.x = x;
	this.y = y;
}

ns.PolygonUtils = function() {
	return {
		triangleContains: function(currentPoint, vertices) {
			//if (!vertices instanceof Array) {console.error("Param vertices is not of type Array.");}
			//if (!currentPoint instanceof Array) {console.error("Param currentPoint is not of type Array.");}

			var p1 = {};
			p1.x = vertices[0][0];
			p1.y = vertices[0][1];

			var p2 = {};
			p2.x = vertices[1][0];
			p2.y = vertices[1][1];

			var p3 = {};
			p3.x = vertices[2][0];
			p3.y = vertices[2][1];

			var alpha = ((p2.y - p3.y) * (currentPoint.x - p3.x) + (p3.x - p2.x) * (currentPoint.y - p3.y)) /
		                ((p2.y - p3.y) * (p1.x - p3.x) + (p3.x - p2.x) * (p1.y - p3.y));
			var beta = ((p3.y - p1.y) * (currentPoint.x - p3.x) + (p1.x - p3.x) * (currentPoint.y - p3.y)) /
			           ((p2.y - p3.y) * (p1.x - p3.x) + (p3.x - p2.x) * (p1.y - p3.y));
			var gamma = 1.0 - alpha - beta;
		
			var barycentricMethod = (alpha > 0 && beta > 0 && gamma > 0) ? true : false;

			if (barycentricMethod) {
				return true;
			}
			else if (this.isPointBetweenTwoOtherPoints(currentPoint, p1, p2) || 
					 this.isPointBetweenTwoOtherPoints(currentPoint, p1, p3) || 
					 this.isPointBetweenTwoOtherPoints(currentPoint, p2, p3)) {
				return true;
			}
			
			return false;
		},

		isPointBetweenTwoOtherPoints: function(currentPoint, p1, p2) {		
			var dxc = currentPoint.x - p1.x;
			var dyc = currentPoint.y - p1.y;
			
			var dx1 = p1.x - p2.x;
			var dy1 = p1.y - p2.y;
			
			var cross = dxc * dy1 - dyc * dx1;
			
			if (cross != 0) {
				return false;	
			}
			else { //checked if it was on vector, now need to check if it's inbetween the two points.
				if (Math.abs(dx1) >= Math.abs(dy1)) {
					return dx1 < 0 ? 
							p1.x <= currentPoint.x && currentPoint.x <= p2.x :
							p2.x <= currentPoint.x && currentPoint.x <= p1.x;
				}
				else {
					return dy1 < 0 ? 
							p1.y <= currentPoint.y && currentPoint.y <= p2.y :
							p2.y <= currentPoint.y && currentPoint.y <= p1.y;
				}
			}
		}
	};
} ();

ns.Node = function (x, y) {
	//var private, this public
	this.x = x;
	this.y = y;
	this.cost = 9999;
	this.parent = null;
	this.neighbors = {};
	this.vertices = [];

	var key = "" + x + y;

	this.getKey = function() {
		return key;
	}

	this.addNeighbor = function(nodeParam) {
		neighbors[nodeObj.getKey()] = nodeObj;
	}
}

ns.NavMesh = function() {
	var nodes = {};

	this.getNodes = function() {
		return nodes;
	}

	this.buildNavMesh = function(objectLayer) {
		//build nodes
		for (var i = 0; i < objectLayer.length; i++) {
			 var triangleMesh = objectLayer[i];
			 var vertices = triangleMesh.polygon

			 for (var v = 0; v < vertices.length; v++) {
		 		vertices[v][0] = vertices[v][0] + triangleMesh.x; 
		 		vertices[v][1] = vertices[v][1] + triangleMesh.y;
			 }

			 var x1 = vertices[0][0];
			 var x2 = vertices[1][0];
			 var x3 = vertices[2][0];
			 var centerX = (x1 + x2 + x3) / 3;

			 var y1 = vertices[0][1];
			 var y2 = vertices[1][1];
			 var y3 = vertices[2][1];
			 var centerY = (y1 + y2 + y3) / 3;

			 var node = new ns.Node(centerX, centerY);
			 node.vertices = vertices;
			 nodes[node.getKey()] = node;
		}

		//add neighbors
		var nodeKeys = Object.keys(nodes);
		for (var k = 0; k < nodeKeys.length; k++) {
			var currentKey = nodeKeys[k];
			var currentNode = nodes[currentKey];
			var currentVertices = currentNode.vertices;

			for (var i = 0; i < nodeKeys.length; i++) {
				var inspectKey = nodeKeys[i];
				var inspectNode = nodes[inspectKey];

				var numSharedVertices = 0;
				var x = 0;
				var y = 1;
				for (var pc = 0; pc < currentVertices.length; pc++) {
					if (ns.PolygonUtils.triangleContains(currentVertices[pc], inspectNode.vertices)) {
						numSharedVertices++;
					}
				}

				if (numSharedVertices >= 2) {
					currentNode.addNeighbor(inspectNode);
				}
			}
		}
	}
}