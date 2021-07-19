
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
    document.getElementById("createFence").disabled = true;

    var geofence = [];
    var lonLatArray = [];
    var mouseLon, mouseLat;
    var tmpLatLonLabel;

    var nodes = [];

    var tmpLatLonLabel = viewer.entities.add({
        label : {
            show : false,
            showBackground : true,
            font : '10px monospace',
            horizontalOrigin : Cesium.HorizontalOrigin.LEFT,
            verticalOrigin : Cesium.VerticalOrigin.TOP,
            pixelOffset : new Cesium.Cartesian2(15, 0)
        }
    });

    boundaryLine = viewer.entities.add({
        name : 'line',
        polyline : {
          positions : undefined,
          material : Cesium.Color.DARKVIOLET.withAlpha(0.8),
          width : 5,
          clampToGround : true,
        },
        show : false
    });

    dummyGuide = viewer.entities.add({
		name : 'Dummy guide line to show mouse movement.',
		polyline : {
			positions : undefined,
			material : Cesium.Color.ROYALBLUE.withAlpha(0.5),
			width : 5,
			clampToGround : true,
		},
		show : false
	});

    boundingArea = viewer.entities.add({
        name: "Bounding Area",
        polygon: {
            hierarchy: undefined,
            material: Cesium.Color.DARKVIOLET.withAlpha(0.3),
            outline: false,
        },
        show: false
    });

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
                pixelSize : 8,
                heightReference : Cesium.HeightReference.CLAMP_TO_GROUND
            },
            show: true
        }));

        lonLatArray.push(lastLon, lastLat);
        geofence.push([lastLat, lastLon]);

        if (nodes.length > 1){
            boundaryLine.polyline.positions = Cesium.Cartesian3.fromDegreesArray(lonLatArray);
            boundaryLine.show = true;

            boundingArea.polygon.hierarchy = Cesium.Cartesian3.fromDegreesArray(lonLatArray);
            boundingArea.show = true;
        };

    }, Cesium.ScreenSpaceEventType.LEFT_CLICK);

    h3.setInputAction(function(click) {
		h1 = h1 && h1.destroy();
		h2 = h2 && h2.destroy();
		h3 = h3 && h3.destroy();

		if (geofence.length == 0)  {
			// The user didn't select any points.

			// Clear alt in object dialogue (if this field exists):
			// if (document.getElementById('object' + objectType + 'AltMSLbaseMeters') != null)  {
			// 	document.getElementById('object' + objectType + 'AltMSLbaseMeters').value = '';
			// };

			// objectPositionsCleanup(utilType, latLonPositions);

		}  else  {
			fenceValue = "GEOFENCE = [";
            for (var i =0; i<geofence.length; i++){
                fenceValue += "[" + geofence[i].toString() + "],\n\t\t";
            }
            fenceValue += "]"
			document.getElementById('geofence').value = fenceValue;
		};
		// Delete the temporary entity:
		tmpLatLonLabel.label.show = false;
		dummyGuide.show = false;

	}, Cesium.ScreenSpaceEventType.RIGHT_CLICK);
};

function copyText(strText) {
	var copyText = document.getElementById(strText);
	copyText.select();
	document.execCommand("copy");
}