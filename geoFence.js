var fenceNodes = [];
var tmpFenceNodes = [];
var boundingArea;

class GeoFence {
    constructor(name, hierarchy) {
        this.name = name;
        this.show = false;

        this.polygon = {
            hierarchy: hierarchy,
            material: Cesium.Color.DARKVIOLET.withAlpha(0.3),
            outline: true,
            outlineColor: Cesium.Color.BlACK
        };
    }
};


function toggleFenceDiv(){
    if (document.getElementById('fenceDiv').style.display == 'none') {
        document.getElementById('fenceIcon').src = "boundary_on.png";

        document.getElementById('btnFenceDiv').style.width = '34px';
        document.getElementById('btnFenceDiv').style.borderTopRightRadius = '0px';
        document.getElementById('btnFenceDiv').style.borderBottomRightRadius = '0px';

        document.getElementById('fenceDiv').style.display = 'block';
    }else{
        document.getElementById('fenceIcon').src = "boundary_off.png";
        document.getElementById('fenceDiv').style.display = 'none';
    }
};

function createFence(){
    tmpFenceNodes = [];

    if (boundingArea) {
        boundingArea.show = false;
        document.getElementById('geofence').value = "";
    }

    document.getElementById("createFence").disabled = true;
    document.getElementById('saveFence').disabled = false;
    document.getElementById('cancelFence').disabled = false;
    document.getElementById("taskMessage").innerHTML = 'Click on map to outline geofence.\nClick "save/export" or right click to finish.';
    document.getElementById('taskdiv').style.display = 'block';

    // var geofence = [];
    var lonLatArray = [];
    var mouseLon, mouseLat;
    var latLonAlt = [];

    var nodes = [];

    h1 = new Cesium.ScreenSpaceEventHandler(viewer.scene.canvas);
	h2 = new Cesium.ScreenSpaceEventHandler(viewer.scene.canvas);
	h3 = new Cesium.ScreenSpaceEventHandler(viewer.scene.canvas);

    h1.setInputAction(function(movement) {
		// We use `viewer.scene.pickPosition` here instead of `viewer.camera.pickEllipsoid` so that
		// we get the correct point when mousing over terrain.
		// var earthPosition = viewer.scene.pickPosition(event.position);
		// `earthPosition` will be undefined if our mouse is not over the globe.
		var mousePosition = viewer.scene.pickPosition(movement.endPosition);
		if (Cesium.defined(mousePosition)) {
			var cartoMouse = Cesium.Cartographic.fromCartesian(mousePosition);
			mouseLon = Cesium.Math.toDegrees(cartoMouse.longitude);
			mouseLat = Cesium.Math.toDegrees(cartoMouse.latitude);
			tmpLatLonLabel.position = mousePosition;
			tmpLatLonLabel.label.show = true;
			tmpLatLonLabel.label.text =
				'Lon: ' + mouseLon.toFixed(4) + '\u00B0' +
				'\nLat: ' + mouseLat.toFixed(4) + '\u00B0';
			tmpLatLonLabel.label.eyeOffset = new Cesium.Cartesian3(0.0, 0.0, -cartoMouse.height * (viewer.scene.mode === Cesium.SceneMode.SCENE2D ? 1.5 : 1.0));
            
            if (nodes.length > 0){
                dummyGuide.polyline.positions = Cesium.Cartesian3.fromDegreesArray([lastLon,lastLat,mouseLon,mouseLat]);
                dummyGuide.show = true;
            }
        
        } else {
			tmpLatLonLabel.label.show = false;
		}
	}, Cesium.ScreenSpaceEventType.MOUSE_MOVE);

    h2.setInputAction(function(click){
        lastLon = mouseLon;
        lastLat = mouseLat;

        nodes.push(viewer.entities.add({
            name: 'node' + nodes.length,
            position: Cesium.Cartesian3.fromDegrees(lastLon, lastLat),
            point : {
                color : Cesium.Color.BLACK,
                pixelSize : 5,
                heightReference : Cesium.HeightReference.CLAMP_TO_GROUND
            },
            show: true
        }));

        lonLatArray.push(lastLon, lastLat);
        tmpFenceNodes.push([lastLat, lastLon]);

        if (nodes.length > 1){
            boundaryLine.polyline.positions = Cesium.Cartesian3.fromDegreesArray(lonLatArray);
            boundaryLine.show = true;

            tmpBoundingArea.polygon.hierarchy = Cesium.Cartesian3.fromDegreesArray(lonLatArray);
            tmpBoundingArea.show = true;
        };

    }, Cesium.ScreenSpaceEventType.LEFT_CLICK);

    h3.setInputAction(function(click) {
		h1 = h1 && h1.destroy();
		h2 = h2 && h2.destroy();
		h3 = h3 && h3.destroy();

		if (tmpFenceNodes.length == 0)  {
			// The user didn't select any points.

			// Clear alt in object dialogue (if this field exists):
			// if (document.getElementById('object' + objectType + 'AltMSLbaseMeters') != null)  {
			// 	document.getElementById('object' + objectType + 'AltMSLbaseMeters').value = '';
			// };

			// objectPositionsCleanup(utilType, latLonPositions);

		}  else  {
            saveFence();
		};
		// Delete the temporary entity:
		tmpLatLonLabel.label.show = false;
		// viewer.entities.remove(dummyGuide);
        dummyGuide.show = false;
        boundaryLine.show = false;
	}, Cesium.ScreenSpaceEventType.RIGHT_CLICK);
};

function saveFence(){
    var fenceInfo = new GeoFence('GeoFence', tmpBoundingArea.polygon.hierarchy);
    
    boundingArea = viewer.entities.add(fenceInfo);
    boundingArea.show = true;
    tmpBoundingArea.show = false;

    fenceNodes = tmpFenceNodes;
    var positions = []
    for (var i=0; i<fenceNodes.length; i++){
        positions.push(Cesium.Cartographic.fromDegrees(fenceNodes[i][1], fenceNodes[i][0]));
    }
    var promise = Cesium.sampleTerrainMostDetailed(viewer.terrainProvider, positions);
    Cesium.when(promise, function(updatedPositions) {
        for (var i = 0; i < fenceNodes.length; i++)  {
            fenceNodes[i].push(positions[i].height);
        }
    });
    document.getElementById('geofence').value = "";
    promise.then(function(){
        writeGeoFence();
    });
    document.getElementById('taskdiv').style.display = 'none';
    document.getElementById('saveFence').disabled = true;
    document.getElementById('cancelFence').disabled = true;
    document.getElementById('createFence').disabled = false;
}

function writeGeoFence(){
    fenceValue = "GEOFENCE = [";
    for (var i =0; i<fenceNodes.length; i++){
        fenceValue += "[" + fenceNodes[i].toString() + "],\n\t\t";
    }
    fenceValue += "]"
    document.getElementById('geofence').value = fenceValue;
}

function cancelFence(){
    if (tmpLatLonLabel) {
        tmpLatLonLabel.label.show = false;
    }
    if (boundaryLine) {
        boundaryLine.show = false;
    }
    if (dummyGuide) {
        dummyGuide.show = false;
    }

    if (boundingArea){
        boundingArea.show = true;
        tmpBoundingArea.show = false;
        writeGeoFence();
    }
    h1.destroy();
    h2.destroy();
    h3.destroy();

    document.getElementById('taskdiv').style.display = 'none';
    document.getElementById('saveFence').disabled = true;
    document.getElementById('cancelFence').disabled = true;
    document.getElementById('createFence').disabled = false;
}

function copyText(strText) {
	var copyText = document.getElementById(strText);
	copyText.select();
	document.execCommand("copy");
}