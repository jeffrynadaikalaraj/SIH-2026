// ==========================================
// CREATE MAP
// ==========================================

const map = L.map("map").setView([13.0827, 80.2707], 16);


// ==========================================
// OPEN STREET MAP
// ==========================================

L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
    attribution: "&copy; OpenStreetMap contributors"
}).addTo(map);


// ==========================================
// GROUND TRUTH PATH
// ==========================================

const groundTruth = [
    [13.0827, 80.2707],
    [13.0830, 80.2710],
    [13.0834, 80.2714],
    [13.0838, 80.2718],
    [13.0842, 80.2722],
    [13.0846, 80.2726],
    [13.0850, 80.2730]
];


// ==========================================
// DEAD RECKONING PATH
// ==========================================

const deadReckoning = [
    [13.0827, 80.2707],
    [13.0830, 80.2711],
    [13.0835, 80.2716],
    [13.0840, 80.2722],
    [13.0845, 80.2728],
    [13.0851, 80.2735],
    [13.0857, 80.2742]
];


// ==========================================
// DRAW GROUND TRUTH
// ==========================================

L.polyline(groundTruth, {
    color: "green",
    weight: 5
}).addTo(map);


// ==========================================
// DRAW DEAD RECKONING
// ==========================================

L.polyline(deadReckoning, {
    color: "blue",
    weight: 5
}).addTo(map);


// ==========================================
// CAR ICON
// ==========================================

const carIcon = L.divIcon({
    className: "car-marker",
    html: "🚗",
    iconSize: [40, 40],
    iconAnchor: [20, 20]
});


// ==========================================
// CREATE VEHICLE
// ==========================================

let vehicleIndex = 0;

const vehicleMarker = L.marker(
    deadReckoning[0],
    {
        icon: carIcon
    }
)
.addTo(map)
.bindPopup("🚗 Current Vehicle Position");


// ==========================================
// MOVE VEHICLE
// ==========================================

setInterval(() => {

    vehicleIndex++;

    if (vehicleIndex >= deadReckoning.length) {
        vehicleIndex = 0;
    }

    vehicleMarker.setLatLng(
        deadReckoning[vehicleIndex]
    );

}, 2000);


// ==========================================
// MAP LEGEND
// ==========================================

const legend = L.control({
    position: "bottomright"
});

legend.onAdd = function () {

    const div = L.DomUtil.create("div", "map-legend");

    div.innerHTML = `
        <div>
            <span class="legend-green"></span>
            Ground Truth
        </div>

        <div>
            <span class="legend-blue"></span>
            Dead Reckoning
        </div>

        <div>
            🚗 Vehicle
        </div>
    `;

    return div;
};

legend.addTo(map);
// ==========================================
// LIVE VEHICLE SPEED
// ==========================================

const speedElement = document.getElementById("vehicleSpeed");

setInterval(() => {

    const speed = Math.floor(
        Math.random() * (60 - 30 + 1) + 30
    );

    speedElement.innerHTML = speed + " km/h";

}, 2000);
// ==========================================
// GNSS STATUS SIMULATION
// ==========================================

const gnssStatus = document.getElementById("gnssStatus");
const navigationMode = document.getElementById("navigationMode");
const outageStatus = document.getElementById("outageStatus");

let gnssTime = 0;

setInterval(() => {

    gnssTime++;

    // GNSS ACTIVE
    if (gnssTime <= 10) {

        gnssStatus.innerHTML = "🟢 ACTIVE";
        navigationMode.innerHTML = "GNSS + IMU";
        outageStatus.innerHTML = "🟢 GNSS ACTIVE";

    }

    // GNSS LOST
    else if (gnssTime <= 20) {

        gnssStatus.innerHTML = "🔴 LOST";
        navigationMode.innerHTML = "DEAD RECKONING";
        outageStatus.innerHTML = "🔴 GNSS LOST";

    }

    // GNSS RESTORED
    else if (gnssTime <= 30) {

        gnssStatus.innerHTML = "🟢 RESTORED";
        navigationMode.innerHTML = "FUSED";
        outageStatus.innerHTML = "🟢 GNSS RESTORED";

    }

    // Restart
    else {

        gnssTime = 0;

    }

}, 1000);
// ==========================================
// LIVE ERROR METRICS
// ==========================================

const positionErrorElement = document.getElementById("positionError");
const maeElement = document.getElementById("mae");
const rmseElement = document.getElementById("rmse");
const driftElement = document.getElementById("drift");

setInterval(() => {

    // GNSS is lost
    if (gnssTime > 10 && gnssTime <= 20) {

        const error = (10 + Math.random() * 15).toFixed(1);
        const mae = (5 + Math.random() * 8).toFixed(1);
        const rmse = (8 + Math.random() * 10).toFixed(1);
        const drift = (1 + Math.random() * 3).toFixed(2);

        positionErrorElement.innerHTML = error + " m";
        maeElement.innerHTML = mae + " m";
        rmseElement.innerHTML = rmse + " m";
        driftElement.innerHTML = drift + " %";
    }

    // GNSS restored
    else if (gnssTime > 20 && gnssTime <= 30) {

        const error = (3 + Math.random() * 5).toFixed(1);
        const mae = (2 + Math.random() * 3).toFixed(1);
        const rmse = (3 + Math.random() * 4).toFixed(1);
        const drift = (0.3 + Math.random() * 1).toFixed(2);

        positionErrorElement.innerHTML = error + " m";
        maeElement.innerHTML = mae + " m";
        rmseElement.innerHTML = rmse + " m";
        driftElement.innerHTML = drift + " %";
    }

}, 1000);
// ==========================================
// BACKEND CONNECTION
// ==========================================

async function getNavigationData() {

    try {

        const response = await fetch("http://127.0.0.1:8000/position");

        const data = await response.json();

        console.log("Backend data:", data);

    } catch (error) {

        console.error("Backend connection failed:", error);

    }
}