// create the map element and set the first view position
var map = L.map('map').setView([35.781736, -81.338296], 15)

// create a tilelayer to display the Google maps sattelite view
var tileLayer = L.tileLayer('https://{s}.google.com/vt/lyrs=s&?x={x}&y={y}&z={z}', {
    maxZoom: 20,
    subdomains: ['mt0', 'mt1', 'mt2', 'mt3']
})
tileLayer.addTo(map)


// the flightPath layer will show a path of the UAV as it flies
var flightPoints = []
var flightPath = L.polyline(flightPoints, {
    color: 'red'
})
flightPath.addTo(map)

// adds a lat and lon location to the flight path polyline
function addFlightPathPoint(lat, lon) {
    flightPoints.push([lat, lon])
    flightPath.setLatLngs(flightPoints)
}

// remove all points from the flight path polyline
function clearFlightPathPoints() {
    flightPoints = []
    flightPath.setLatLngs(flightPoints)
}


// the detection layer will hold many markers that represent the location of objects detected by the Jetson Nano
var detectionMarkers = []
var detectionLayer = L.layerGroup()
detectionLayer.addTo(map)

// adds a detection marker at a location and with the detection probability
function addDetectionMarker(probability, lat, lon) {
    // create a marker at the location
    var marker = L.marker([lat, lon])

    // set the popup content to show the probability of being a person, and the exact location
    marker.bindPopup('<b>Person:</b> ' + Number(probability * 100).toFixed(2) + '%<br><b>Lat:</b> ' + lat + '<br><b>Lon:</b> ' + lon)

    // make the popup show and hide when user mouses over
    marker.on('mouseover', () => {
        marker.openPopup()
    })
    marker.on('mouseout', () => {
        marker.closePopup()
    })

    // add marker to the detectionLayer to show on the map
    detectionMarkers.push(marker)
    detectionLayer.addLayer(marker)
}



// add layer control so user can show/hide various layers
L.control.layers({
    'Google': tileLayer
}, {
    'Detection Results': detectionLayer,
    'Flight': flightPath
}).addTo(map)

// show the map scale for better understanding
L.control.scale().addTo(map)