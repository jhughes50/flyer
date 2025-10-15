// Initialize map with center coordinates and appropriate zoom level
var map = L.map('map').setView([39.941326, -75.199492], 16);

// Try different tile providers - one of these should work
// 1. OpenStreetMap Standard
var mapboxDetailed = L.tileLayer('https://api.mapbox.com/styles/v1/{id}/tiles/{z}/{x}/{y}?access_token={accessToken}', {
    attribution: '© <a href="https://www.mapbox.com/about/maps/">Mapbox</a>',
    maxZoom: 30,
    id: 'mapbox/satellite-v9',
    accessToken: '' //ADD A MAPBOX TOKEN HERE
}).addTo(map);

var baseLayers = {
    "MapBox": mapboxDetailed
};

L.control.layers(baseLayers).addTo(map);

// Add a console log to check if the map initialized
console.log("Map initialized with center:", map.getCenter(), "zoom:", map.getZoom());

// Create layers for your data
var pointsLayer = L.layerGroup().addTo(map);
var pathLayer = L.layerGroup().addTo(map);
var objectsLayer = L.layerGroup().addTo(map);
var connectionsLayer = L.layerGroup().addTo(map);

// Create a legend
var legend = L.control({position: 'bottomright'});

legend.onAdd = function (map) {
    var div = L.DomUtil.create('div', 'info legend');
    div.style.backgroundColor = 'rgba(255, 255, 255, 0.8)';
    div.style.padding = '10px';
    div.style.borderRadius = '5px';
    div.style.boxShadow = '0 0 15px rgba(0,0,0,0.2)';
    
    // Add title
    div.innerHTML = '<h4 style="margin-top: 0; margin-bottom: 10px; text-align: center;">Legend</h4>';
    
    // Define your legend items here
    // Format: [color, label]
    var items = [
        ['#FF3333', 'UAV Position'],
        // Add your object types here with their colors
        // For example:
        ['#28B463', 'Objects'],
        ['#2980B9', 'Roads'],
    ];
    
    // Collect unique colors from objects to populate legend dynamically
    var objectColors = {};
    
    // Function to update legend with object colors
    window.updateLegendWithObjectColors = function(objects) {
        if (!objects || objects.length === 0) return;
        
        objects.forEach(function(obj) {
            if (obj.color && obj.label) {
                objectColors[obj.color] = obj.label;
            }
        });
        
        // Rebuild items array
        items = [['#FF3333', 'Current Position/Track']];
        
        // Add object colors to items
        for (var color in objectColors) {
            items.push([color, objectColors[color]]);
        }
        
        // Update legend HTML
        updateLegendHTML();
    };
    
    // Function to update legend HTML
    function updateLegendHTML() {
        var legendHTML = '<h4 style="margin-top: 0; margin-bottom: 10px; text-align: center;">Legend</h4>';
        
        // Add each legend item
        for (var i = 0; i < items.length; i++) {
            legendHTML += 
                '<div style="display: flex; align-items: center; margin-bottom: 5px;">' +
                    '<span style="background:' + items[i][0] + '; width: 15px; height: 15px; border-radius: 50%; display: inline-block; margin-right: 5px; border: 1px solid #FFF;"></span> ' +
                    '<span>' + items[i][1] + '</span>' +
                '</div>';
        }
        
        div.innerHTML = legendHTML;
    }
    
    // Initial legend HTML update
    updateLegendHTML();
    
    return div;
};

// Add legend to map
legend.addTo(map);

// Connect to WebSocket
var socket = io();

// Listen for GPS updates
socket.on('gps_update', function(data) {
    console.log("Received GPS update:", data);
    
    // Clear previous markers and paths
    pointsLayer.clearLayers();
    pathLayer.clearLayers();
    
    if (data.points.length === 0) return;
    
    // Set the color to use for GPS track
    var trackColor = '#FF3333';  // Red color
    
    // Extract all coordinates for the path
    var pathCoords = data.points.map(point => [point.lat, point.lon]);
    
    // Draw the connecting line for all points
    if (pathCoords.length > 1) {
        L.polyline(pathCoords, {
            color: trackColor,
            weight: 3,
            opacity: 0.7
        }).addTo(pathLayer);
    }
    
    // Add only the most recent point as a red circle marker
    var lastPoint = data.points[data.points.length - 1];
    
    // Use circleMarker instead of standard marker
    L.circleMarker([lastPoint.lat, lastPoint.lon], {
        radius: 8,
        fillColor: trackColor,
        color: '#FFFFFF',  // White border
        weight: 2,
        opacity: 1,
        fillOpacity: 1
    })
    .bindPopup(lastPoint.popup || "Current Position")
    .addTo(pointsLayer);
});

// Listen for object updates
socket.on('objects_update', function(data) {
    console.log("Received objects update:", data);
    
    // Clear previous objects and connections
    objectsLayer.clearLayers();
    connectionsLayer.clearLayers();
    
    if (!data.objects || data.objects.length === 0) {
        console.log("No objects to display");
        return;
    }
    
    // Debug: Log the full objects data including connections
    console.log("Full objects data:", JSON.stringify(data.objects, null, 2));
    
    // Create an array to store object references by index
    var objectsByIndex = data.objects;
    
    // Also create a map of objects by label for easier lookup
    var objectsByLabel = {};
    data.objects.forEach(function(obj, index) {
        objectsByLabel[obj.label] = obj;
    });
    
    // Store all markers for later connection drawing
    var allMarkers = {};
    
    // Add objects as circle markers
    data.objects.forEach(function(obj, index) {
        var marker = L.circleMarker([obj.lat, obj.lon], {
            radius: 8,
            fillColor: obj.color || '#28B463',  // Default to green if no color specified
            color: '#FFFFFF',
            weight: 2,
            opacity: 1,
            fillOpacity: 1
        })
        .bindPopup(obj.label || "Object")
        .addTo(objectsLayer);
        
        // Store marker reference by both index and label for connections
        allMarkers[index] = {
            marker: marker,
            obj: obj
        };
        
        if (obj.label) {
            allMarkers[obj.label] = {
                marker: marker,
                obj: obj
            };
        }
    });
    
    // Now draw all connections - try both index-based and label-based connections
    data.objects.forEach(function(obj, srcIndex) {
        // Check if this object has connections
        if (obj.connections && obj.connections.length > 0) {
            console.log(`Drawing connections for ${obj.label} (index ${srcIndex}):`, obj.connections);
            
            // For each connection
            obj.connections.forEach(function(connection) {
                // Connection might be an index or a label
                let connObj = null;
                let targetIndex = parseInt(connection);
                
                // Try to find the target object by index first
                if (!isNaN(targetIndex) && objectsByIndex[targetIndex]) {
                    connObj = objectsByIndex[targetIndex];
                    console.log(`Found connection by index ${targetIndex}: ${connObj.label}`);
                } 
                // Then try by label
                else if (objectsByLabel[connection]) {
                    connObj = objectsByLabel[connection];
                    console.log(`Found connection by label ${connection}`);
                }
                
                // If we found the connected object, draw a line
                if (connObj) {
                    console.log(`Drawing line from ${obj.lat},${obj.lon} to ${connObj.lat},${connObj.lon}`);
                    
                    // Draw the connection line
                    L.polyline([
                        [obj.lat, obj.lon],
                        [connObj.lat, connObj.lon]
                    ], {
                        color: obj.color || '#28B463',
                        weight: 3,
                        opacity: 0.9,
                        dashArray: '5, 5'  // Dashed line
                    }).addTo(connectionsLayer);
                } else {
                    console.log(`⚠️ Could not find connection target: ${connection}`);
                }
            });
        } else {
            console.log(`Object ${obj.label} has no connections or empty connections array`);
        }
    }); 
    }
});

socket.on('orientation_update', function(data) {
    console.log("Received orientation update:", data);
    
    // Update the RPY display with the new values
    if (data.roll !== undefined) {
        document.getElementById('roll-value').textContent = data.roll.toFixed(2) + '°';
    }
    
    if (data.pitch !== undefined) {
        document.getElementById('pitch-value').textContent = data.pitch.toFixed(2) + '°';
    }
    
    if (data.yaw !== undefined) {
        document.getElementById('yaw-value').textContent = data.yaw.toFixed(2) + '°';
    }
});
