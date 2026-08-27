/**
 * app.js
 * ======
 * MapX — Google Maps Style Real-Time Navigation & Dead Reckoning System
 * Features:
 * - 100% Watermark-free OpenStreetMap / Humanitarian Tiles
 * - Real GPS Geolocation & Dynamic Landmark Routing
 * - Intelligent Auto-Recenter Engine with Floating Re-center Pill
 * - Dynamic Turn-by-Turn Maneuver Guidance ("towards Gandhi Road" & "Then ↱")
 * - Isolated ISRO PS 26168 Dead Reckoning Engineering Evaluation Suite
 */

// =============================================================================
// DEFAULT CONFIGURATION & STATE
// =============================================================================

const CONFIG = {
    // Default Chennai / Gandhi Road coordinates
    DEFAULT_ORIGIN: { lat: 13.0827, lon: 80.2707, name: "My GPS Location", address: "Chennai, Tamil Nadu" },
    DEFAULT_DEST: { lat: 13.0915, lon: 80.2820, name: "Gandhi Road", address: "Anna Nagar / Central Corridor" },
    WS_URL: `${window.location.protocol === 'https:' ? 'wss:' : 'ws:'}//${window.location.host}/navigation/live`,
    POLL_INTERVAL_MS: 150
};

const appState = {
    isNavigating: false,
    is3DView: false,
    autoFollow: true,       // Controls whether camera automatically tracks vehicle
    voiceEnabled: false,
    userGpsLocation: null,
    currentOrigin: { ...CONFIG.DEFAULT_ORIGIN },
    currentDestination: { ...CONFIG.DEFAULT_DEST },
    activeRoute: null,
    outagePath: [],
    vehiclePos: [CONFIG.DEFAULT_ORIGIN.lat, CONFIG.DEFAULT_ORIGIN.lon],
    vehicleHeading: 0,
    vehicleSpeedKmh: 45,
    gnssStatus: "GNSS_ACTIVE",
    activeStep: null,
    currentRoadName: "Gandhi Road",
    metrics: {
        currentError: 0,
        mae: 0,
        rmse: 0,
        driftPercentage: 0,
        distanceTravelled: 0,
        outageActive: false
    }
};

// =============================================================================
// DOM ELEMENT REFERENCES
// =============================================================================

const DOM = {
    appContainer: document.getElementById("app"),
    mapViewport: document.getElementById("map-viewport"),
    
    // Top Bar & Search
    topSearchBar: document.getElementById("top-search-bar"),
    destinationInput: document.getElementById("destination-input"),
    searchSuggestions: document.getElementById("search-suggestions"),
    locateMeBtn: document.getElementById("locate-me-btn"),
    clearSearchBtn: document.getElementById("clear-search-btn"),
    categoryChips: document.querySelectorAll(".chip-btn"),
    menuBtn: document.getElementById("menu-btn"),

    // Driving Banner (Google Maps Teal)
    navigationBanner: document.getElementById("navigation-banner"),
    navManeuverIcon: document.getElementById("nav-maneuver-icon"),
    navTargetRoad: document.getElementById("nav-target-road"),
    navInstructionText: document.getElementById("nav-instruction-text"),
    navThenPill: document.getElementById("nav-then-pill"),
    thenManeuverIcon: document.getElementById("then-maneuver-icon"),

    // Speedometer & HUD
    speedometerHud: document.getElementById("speedometer-hud"),
    liveSpeedVal: document.getElementById("live-speed-val"),
    gnssStatusBadge: document.getElementById("gnss-status-badge"),
    gnssStatusLabel: document.getElementById("gnss-status-label"),

    // Re-center Pill & Controls Stack
    recenterPillWrapper: document.getElementById("recenter-pill-wrapper"),
    recenterMainBtn: document.getElementById("recenter-main-btn"),
    mapControlsStack: document.getElementById("map-controls-stack"),
    compassBtn: document.getElementById("compass-btn"),
    compassNeedle: document.getElementById("compass-needle"),
    toggle3dBtn: document.getElementById("toggle-3d-btn"),
    soundToggleBtn: document.getElementById("sound-toggle-btn"),
    recenterBtn: document.getElementById("recenter-btn"),
    quickGnssOutageBtn: document.getElementById("quick-gnss-outage-btn"),

    // Bottom Navigation Sheet
    bottomSheet: document.getElementById("bottom-sheet"),
    routePreviewPanel: document.getElementById("route-preview-panel"),
    activeDrivingPanel: document.getElementById("active-driving-panel"),
    previewDestTitle: document.getElementById("preview-dest-title"),
    previewDestAddress: document.getElementById("preview-dest-address"),
    previewEtaDuration: document.getElementById("preview-eta-duration"),
    previewEtaDistance: document.getElementById("preview-eta-distance"),
    startNavBtn: document.getElementById("start-nav-btn"),
    previewStepsBtn: document.getElementById("preview-steps-btn"),
    exitNavBtn: document.getElementById("exit-nav-btn"),
    driveRemainingTime: document.getElementById("drive-remaining-time"),
    driveRemainingDist: document.getElementById("drive-remaining-dist"),
    driveEtaClock: document.getElementById("drive-eta-clock"),
    driveMetricsBtn: document.getElementById("drive-metrics-btn"),

    // Drawer Menu
    sideDrawer: document.getElementById("side-drawer"),
    menuOverlay: document.getElementById("menu-overlay"),
    closeDrawerBtn: document.getElementById("close-drawer-btn"),
    menuNavBtn: document.getElementById("menu-nav-btn"),
    menuGpsBtn: document.getElementById("menu-gps-btn"),
    menuSavedBtn: document.getElementById("menu-saved-btn"),
    menuMetricsBtn: document.getElementById("menu-metrics-btn"),

    // System Metrics Modal
    metricsModal: document.getElementById("metrics-modal"),
    closeMetricsBtn: document.getElementById("close-metrics-btn"),
    metricCurrentErr: document.getElementById("metric-current-err"),
    metricMae: document.getElementById("metric-mae"),
    metricRmse: document.getElementById("metric-rmse"),
    metricDrift: document.getElementById("metric-drift"),
    outage15sBtn: document.getElementById("outage-15s-btn"),
    outage30sBtn: document.getElementById("outage-30s-btn"),
    outageToggleBtn: document.getElementById("outage-toggle-btn"),
    outageRestoreBtn: document.getElementById("outage-restore-btn")
};

// =============================================================================
// MAP INITIALIZATION (High-Performance Watermark-Free OpenStreetMap Layer)
// =============================================================================

let map;
let vehicleMarker;
let destinationMarker;
let routePolyline;
let drPolyline;
let errorChart;
let webSocketConnection;
let pollingIntervalTimer;

function initMap() {
    map = L.map("map", {
        zoomControl: false,
        attributionControl: false,
        fadeAnimation: true,
        markerZoomAnimation: true
    }).setView([appState.currentOrigin.lat, appState.currentOrigin.lon], 16);

    // 100% Free, High-Resolution, Watermark-Free OpenStreetMap Tiles
    L.tileLayer("https://{s}.tile.openstreetmap.fr/hot/{z}/{x}/{y}.png", {
        subdomains: ['a', 'b', 'c'],
        maxZoom: 19
    }).addTo(map);

    // Google Maps Style Blue Directional Vehicle Marker
    const vehicleIconHtml = `
        <div class="google-vehicle-marker" id="google-vehicle-elem">
            <div class="vehicle-street-pill" id="vehicle-road-badge">Gandhi Road</div>
            <div class="vehicle-blue-halo"></div>
            <svg class="vehicle-arrow-icon" id="vehicle-arrow-svg" viewBox="0 0 36 36">
                <circle cx="18" cy="18" r="14" fill="#ffffff" filter="drop-shadow(0 2px 4px rgba(0,0,0,0.35))"/>
                <polygon points="18,7 26,27 18,22 10,27" fill="#1a73e8"/>
            </svg>
        </div>
    `;

    const vehicleCustomIcon = L.divIcon({
        className: 'vehicle-div-icon',
        html: vehicleIconHtml,
        iconSize: [44, 44],
        iconAnchor: [22, 22]
    });

    vehicleMarker = L.marker([appState.currentOrigin.lat, appState.currentOrigin.lon], {
        icon: vehicleCustomIcon,
        zIndexOffset: 1000
    }).addTo(map);

    // Destination Pin
    const destIconHtml = `<div class="dest-pin-marker"><span class="pin-icon">📍</span></div>`;
    const destCustomIcon = L.divIcon({
        className: 'dest-div-icon',
        html: destIconHtml,
        iconSize: [32, 32],
        iconAnchor: [16, 30]
    });
    destinationMarker = L.marker([appState.currentDestination.lat, appState.currentDestination.lon], {
        icon: destCustomIcon
    }).addTo(map);

    // Google Maps Blue Driving Route Polyline
    routePolyline = L.polyline([], {
        color: '#1a73e8',
        weight: 7,
        opacity: 0.95,
        lineCap: 'round',
        lineJoin: 'round'
    }).addTo(map);

    // Red Dead Reckoning segment Polyline
    drPolyline = L.polyline([], {
        color: '#ea4335',
        weight: 7,
        opacity: 0.95,
        lineCap: 'round',
        lineJoin: 'round'
    }).addTo(map);

    // User Map Interaction Listeners (Detect manual drag -> show Re-center pill)
    map.on('dragstart', () => {
        if (appState.isNavigating) {
            appState.autoFollow = false;
            DOM.recenterPillWrapper.classList.remove("hidden");
        }
    });
}

// =============================================================================
// RE-CENTER ENGINE
// =============================================================================

function triggerRecenter() {
    appState.autoFollow = true;
    DOM.recenterPillWrapper.classList.add("hidden");
    map.setView(appState.vehiclePos, 17, { animate: true, duration: 0.35 });
    
    // Reset compass rotation to vehicle heading
    DOM.compassNeedle.style.transform = `rotate(${appState.vehicleHeading}deg)`;
}

// =============================================================================
// REAL-TIME GPS GEOLOCATION ENGINE
// =============================================================================

function detectRealUserGpsLocation(andCalculateRoute = true) {
    if (!("geolocation" in navigator)) {
        console.warn("Geolocation API not supported.");
        return;
    }

    DOM.locateMeBtn.classList.add("active");

    navigator.geolocation.getCurrentPosition(
        (position) => {
            const lat = position.coords.latitude;
            const lon = position.coords.longitude;
            appState.userGpsLocation = { lat, lon };
            appState.currentOrigin = {
                lat,
                lon,
                name: "My GPS Location",
                address: `${lat.toFixed(4)}, ${lon.toFixed(4)}`
            };

            updateVehicleMarker(lat, lon, 0, "My Location");
            map.setView([lat, lon], 16);
            DOM.locateMeBtn.classList.remove("active");

            if (andCalculateRoute) {
                appState.currentDestination = {
                    lat: lat + 0.012,
                    lon: lon + 0.010,
                    name: "Gandhi Road",
                    address: "Main Corridor"
                };
                requestRoute(appState.currentOrigin, appState.currentDestination);
            }
        },
        (error) => {
            console.warn("Geolocation permission error:", error.message);
            DOM.locateMeBtn.classList.remove("active");
        },
        { enableHighAccuracy: true, timeout: 8000, maximumAge: 10000 }
    );
}

// =============================================================================
// ROUTING & NAVIGATION LIFECYCLE
// =============================================================================

async function requestRoute(origin, destination) {
    try {
        const payload = {
            origin_lat: origin.lat,
            origin_lon: origin.lon,
            dest_lat: destination.lat,
            dest_lon: destination.lon
        };

        const response = await fetch("/navigation/route", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify(payload)
        });

        if (!response.ok) throw new Error("Failed to calculate route");

        const routeData = await response.json();
        appState.activeRoute = routeData;

        // Render route line
        const coords = routeData.geometry;
        routePolyline.setLatLngs(coords);

        destinationMarker.setLatLng([destination.lat, destination.lon]);
        map.fitBounds(routePolyline.getBounds(), { padding: [60, 60] });

        // Update Preview Panel Info
        const durMin = Math.max(1, Math.round(routeData.duration_sec / 60));
        const distKm = (routeData.distance_m / 1000).toFixed(1);

        DOM.previewDestTitle.textContent = destination.name;
        DOM.previewDestAddress.textContent = destination.address || "Driving Route";
        DOM.previewEtaDuration.textContent = `${durMin} min`;
        DOM.previewEtaDistance.textContent = `(${distKm} km)`;
        DOM.startNavBtn.disabled = false;

        return routeData;
    } catch (err) {
        console.error("Route error:", err);
    }
}

async function startNavigation() {
    if (!appState.activeRoute) return;

    try {
        const payload = {
            origin_lat: appState.currentOrigin.lat,
            origin_lon: appState.currentOrigin.lon,
            dest_lat: appState.currentDestination.lat,
            dest_lon: appState.currentDestination.lon,
            origin_name: appState.currentOrigin.name,
            destination_name: appState.currentDestination.name,
            speed_multiplier: 1.0
        };

        const response = await fetch("/navigation/start", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify(payload)
        });

        if (!response.ok) throw new Error("Could not start navigation session");

        appState.isNavigating = true;
        appState.autoFollow = true;
        appState.outagePath = [];
        if (drPolyline) drPolyline.setLatLngs([]);

        // Transition UI to Driving Mode
        DOM.appContainer.classList.add("nav-active-mode");
        DOM.topSearchBar.style.opacity = '0';
        DOM.topSearchBar.style.pointerEvents = 'none';
        DOM.navigationBanner.classList.remove("hidden");
        DOM.routePreviewPanel.classList.add("hidden");
        DOM.activeDrivingPanel.classList.remove("hidden");
        DOM.recenterPillWrapper.classList.add("hidden");

        set3DView(true);
        map.setZoom(17);
        map.panTo(appState.vehiclePos, { animate: true });

        speakGuidance(`Head towards ${appState.currentDestination.name}. Navigation started.`);

        connectWebSocket();

    } catch (err) {
        console.error("Start navigation error:", err);
    }
}

function exitNavigation() {
    appState.isNavigating = false;
    appState.autoFollow = true;
    appState.outagePath = [];
    if (drPolyline) drPolyline.setLatLngs([]);

    // Restore Preview UI
    DOM.appContainer.classList.remove("nav-active-mode");
    DOM.topSearchBar.style.opacity = '1';
    DOM.topSearchBar.style.pointerEvents = 'auto';
    DOM.navigationBanner.classList.add("hidden");
    DOM.routePreviewPanel.classList.remove("hidden");
    DOM.activeDrivingPanel.classList.add("hidden");
    DOM.recenterPillWrapper.classList.add("hidden");

    set3DView(false);
    const mapElem = document.getElementById("map");
    if (mapElem) mapElem.style.transform = "";

    if (webSocketConnection) webSocketConnection.close();
    if (pollingIntervalTimer) clearInterval(pollingIntervalTimer);

    map.setView([appState.currentOrigin.lat, appState.currentOrigin.lon], 16);
}

// =============================================================================
// REAL-TIME STATE SYNC & VEHICLE MARKER TRACKING
// =============================================================================

function connectWebSocket() {
    try {
        webSocketConnection = new WebSocket(CONFIG.WS_URL);

        webSocketConnection.onmessage = (event) => {
            const data = JSON.parse(event.data);
            if (data.type === "LIVE_STATE") {
                // Clear active HTTP polling if WebSocket is successful
                if (pollingIntervalTimer) {
                    clearInterval(pollingIntervalTimer);
                    pollingIntervalTimer = null;
                }
                handleLiveStateUpdate(data.state, data.metrics);
            }
        };

        webSocketConnection.onerror = () => startHttpPolling();
        webSocketConnection.onclose = () => {
            if (appState.isNavigating) setTimeout(connectWebSocket, 2000);
        };
    } catch (e) {
        startHttpPolling();
    }
}

function startHttpPolling() {
    if (pollingIntervalTimer) clearInterval(pollingIntervalTimer);
    pollingIntervalTimer = setInterval(async () => {
        if (!appState.isNavigating) return;
        try {
            // Centralized simulation loop on backend handles stepping;
            // we only need to fetch the latest state and metrics.
            const [stateRes, metricsRes] = await Promise.all([
                fetch("/navigation/state"),
                fetch("/navigation/metrics")
            ]);
            const state = await stateRes.json();
            const metrics = await metricsRes.json();
            handleLiveStateUpdate(state, metrics);
        } catch (err) {
            console.error("Polling error:", err);
        }
    }, CONFIG.POLL_INTERVAL_MS);
}

function handleLiveStateUpdate(state, metrics) {
    if (!state) return;

    const lat = state.latitude;
    const lon = state.longitude;
    const heading = state.heading_deg || 0;
    const speed = state.speed_kmh || 0;
    const roadName = state.current_road || "Gandhi Road";

    appState.vehiclePos = [lat, lon];
    appState.vehicleHeading = heading;
    appState.vehicleSpeedKmh = speed;
    appState.gnssStatus = state.gnss_status;
    appState.currentRoadName = roadName;

    // Update vehicle position and road badge
    updateVehicleMarker(lat, lon, heading, roadName);

    // Auto-center camera if autoFollow is active
    if (appState.isNavigating && appState.autoFollow) {
        map.panTo([lat, lon], { animate: false });
    }

    // Rotate map according to vehicle heading
    updateMapCameraRotation(heading);

    // Update Driving Turn Banner
    updateTurnBanner(state);

    // Update Speedometer & GNSS Badge
    DOM.liveSpeedVal.textContent = Math.round(speed);
    if (state.gnss_status === "GNSS_LOST") {
        DOM.gnssStatusBadge.className = "gnss-badge gnss-lost";
        DOM.gnssStatusLabel.textContent = "DEAD RECKONING";

        // Track dead reckoning path
        if (!appState.outagePath) appState.outagePath = [];
        const lastPt = appState.outagePath[appState.outagePath.length - 1];
        if (!lastPt || lastPt[0] !== lat || lastPt[1] !== lon) {
            appState.outagePath.push([lat, lon]);
            if (drPolyline) drPolyline.setLatLngs(appState.outagePath);
        }
    } else {
        DOM.gnssStatusBadge.className = "gnss-badge gnss-active";
        DOM.gnssStatusLabel.textContent = "GPS ACTIVE";

        // Clear DR segment when GNSS is restored
        if (appState.outagePath && appState.outagePath.length > 0) {
            appState.outagePath = [];
            if (drPolyline) drPolyline.setLatLngs([]);
        }
    }

    // Update heading text
    const headingElem = document.getElementById("live-heading-val");
    if (headingElem) {
        const sector = getHeadingSector(heading);
        headingElem.textContent = `Heading ${sector.label} ${sector.arrow}`;
    }

    // Update Bottom Dark Driving Sheet (ETA, distance, clock)
    const remMin = Math.max(1, Math.round((state.remaining_duration_sec || 0) / 60));
    const remKm = ((state.remaining_distance_m || 0) / 1000).toFixed(1);
    
    const etaDate = new Date(Date.now() + (state.remaining_duration_sec || 0) * 1000);
    const etaString = etaDate.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' }).toLowerCase();

    DOM.driveRemainingTime.textContent = `${remMin} min`;
    DOM.driveRemainingDist.textContent = `${remKm} km`;
    DOM.driveEtaClock.textContent = etaString;

    // Update Metrics
    if (metrics) {
        appState.metrics = metrics;
        DOM.metricCurrentErr.textContent = `${(metrics.current_error_m || 0).toFixed(1)} m`;
        DOM.metricMae.textContent = `${(metrics.mae_m || 0).toFixed(1)} m`;
        DOM.metricRmse.textContent = `${(metrics.rmse_m || 0).toFixed(1)} m`;
        DOM.metricDrift.textContent = `${(metrics.drift_percentage || 0).toFixed(2)} %`;

        if (errorChart) pushChartData(metrics.current_error_m || 0);
    }
}

function getHeadingSector(deg) {
    deg = (deg % 360 + 360) % 360;
    const sectors = [
        { label: "North", arrow: "↑" },
        { label: "Northeast", arrow: "↗" },
        { label: "East", arrow: "→" },
        { label: "Southeast", arrow: "↘" },
        { label: "South", arrow: "↓" },
        { label: "Southwest", arrow: "↙" },
        { label: "West", arrow: "←" },
        { label: "Northwest", arrow: "↖" }
    ];
    const index = Math.round(deg / 45) % 8;
    return sectors[index];
}

function updateMapCameraRotation(heading) {
    const mapElem = document.getElementById("map");
    if (mapElem) {
        if (appState.isNavigating && appState.autoFollow) {
            mapElem.style.transform = `rotate(${-heading}deg)`;
            mapElem.style.transformOrigin = "50% 50%";
        } else {
            mapElem.style.transform = "";
        }
    }
}

function updateVehicleMarker(lat, lon, heading, roadName = "Gandhi Road") {
    vehicleMarker.setLatLng([lat, lon]);
    
    const roadBadge = document.getElementById("vehicle-road-badge");
    if (roadBadge) {
        roadBadge.textContent = roadName;
        if (appState.isNavigating && appState.autoFollow) {
            roadBadge.style.transform = `rotate(${heading}deg)`;
        } else {
            roadBadge.style.transform = "";
        }
    }

    const arrowSvg = document.getElementById("vehicle-arrow-svg");
    if (arrowSvg) {
        arrowSvg.style.transform = `rotate(${heading}deg)`;
    }

    // Update 3D Compass
    DOM.compassNeedle.style.transform = `rotate(${heading}deg)`;
}

function set3DView(enable3D) {
    appState.is3DView = enable3D;
    if (enable3D) {
        DOM.mapViewport.classList.add("map-3d-driving");
        DOM.toggle3dBtn.classList.add("active");
    } else {
        DOM.mapViewport.classList.remove("map-3d-driving");
        DOM.toggle3dBtn.classList.remove("active");
    }
}

function updateTurnBanner(state) {
    const road = state.current_road || "Gandhi Road";
    DOM.navTargetRoad.textContent = road;
    DOM.navInstructionText.textContent = state.instruction || `towards ${road}`;
    DOM.navManeuverIcon.innerHTML = getManeuverSvg(state.maneuver_icon);
}

function getManeuverSvg(iconType) {
    switch (iconType) {
        case "turn-right":
            return `<svg viewBox="0 0 24 24" width="38" height="38" fill="currentColor"><path d="M17.5 13H9.5V20H6.5V10c0-1.1.9-2 2-2h9V4.5l5.5 5.5-5.5 5.5V13z"/></svg>`;
        case "turn-left":
            return `<svg viewBox="0 0 24 24" width="38" height="38" fill="currentColor"><path d="M6.5 13h8V20h3V10c0-1.1-.9-2-2-2h-9V4.5L1 10l5.5 5.5V13z"/></svg>`;
        case "slight-right":
            return `<svg viewBox="0 0 24 24" width="38" height="38" fill="currentColor"><path d="M14 4l3.5 3.5-5 5V20h-3v-9l6.5-6.5L14 4z"/></svg>`;
        case "slight-left":
            return `<svg viewBox="0 0 24 24" width="38" height="38" fill="currentColor"><path d="M10 4L6.5 7.5l5 5V20h3v-9l-6.5-6.5L10 4z"/></svg>`;
        case "arrive":
            return `<svg viewBox="0 0 24 24" width="38" height="38" fill="currentColor"><path d="M12 2C8.13 2 5 5.13 5 9c0 5.25 7 13 7 13s7-7.75 7-13c0-3.87-3.13-7-7-7zm0 9.5a2.5 2.5 0 0 1 0-5 2.5 2.5 0 0 1 0 5z"/></svg>`;
        default:
            return `<svg viewBox="0 0 24 24" width="38" height="38" fill="currentColor"><path d="M12 2L4.5 10.5h5V20h5V10.5h5L12 2z"/></svg>`;
    }
}

// =============================================================================
// VOICE GUIDANCE
// =============================================================================

function speakGuidance(text) {
    if (!appState.voiceEnabled || !('speechSynthesis' in window)) return;
    window.speechSynthesis.cancel();
    const utterance = new SpeechSynthesisUtterance(text);
    utterance.rate = 1.05;
    window.speechSynthesis.speak(utterance);
}

// =============================================================================
// DESTINATION SEARCH & AUTOCOMPLETE
// =============================================================================

let searchDebounceTimer;

async function performSearch(query) {
    try {
        const res = await fetch(`/destinations/search?q=${encodeURIComponent(query)}`);
        const data = await res.json();
        renderSuggestions(data.results || []);
    } catch (e) {
        console.error("Search error:", e);
    }
}

function renderSuggestions(results) {
    DOM.searchSuggestions.innerHTML = "";
    if (!results.length) {
        DOM.searchSuggestions.classList.add("hidden");
        return;
    }

    results.forEach(item => {
        const row = document.createElement("div");
        row.className = "search-suggestion-item";
        row.innerHTML = `
            <span class="suggestion-icon">${item.icon || '📍'}</span>
            <div class="suggestion-text">
                <span class="suggestion-name">${item.name}</span>
                <span class="suggestion-address">${item.address}</span>
            </div>
        `;
        row.addEventListener("click", () => selectDestination(item));
        DOM.searchSuggestions.appendChild(row);
    });

    DOM.searchSuggestions.classList.remove("hidden");
}

function selectDestination(item) {
    appState.currentDestination = {
        lat: item.latitude,
        lon: item.longitude,
        name: item.name,
        address: item.address
    };

    DOM.destinationInput.value = item.name;
    DOM.searchSuggestions.classList.add("hidden");
    DOM.clearSearchBtn.classList.remove("hidden");

    requestRoute(appState.currentOrigin, appState.currentDestination);
}

// =============================================================================
// GNSS OUTAGE SIMULATOR (DEAD RECKONING MODE)
// =============================================================================

async function triggerGnssLoss(durationSec = 25.0) {
    try {
        const res = await fetch("/navigation/simulate-gnss-loss", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ duration_sec: durationSec })
        });
        const data = await res.json();
        console.log("GNSS Outage Triggered (Dead Reckoning active):", data);
    } catch (err) {
        console.error("Error triggering GNSS loss:", err);
    }
}

async function restoreGnss() {
    try {
        const res = await fetch("/navigation/restore-gnss", { method: "POST" });
        const data = await res.json();
        console.log("GNSS Restored:", data);
    } catch (err) {
        console.error("Error restoring GNSS:", err);
    }
}

function initErrorChart() {
    const ctx = document.getElementById("error-chart").getContext("2d");
    errorChart = new Chart(ctx, {
        type: 'line',
        data: {
            labels: Array(30).fill(''),
            datasets: [{
                label: 'Position Error (m)',
                data: Array(30).fill(0),
                borderColor: '#1a73e8',
                backgroundColor: 'rgba(26, 115, 232, 0.1)',
                borderWidth: 2,
                fill: true,
                tension: 0.3,
                pointRadius: 0
            }]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            plugins: { legend: { display: false } },
            scales: {
                x: { display: false },
                y: {
                    beginAtZero: true,
                    suggestedMax: 15,
                    grid: { color: 'rgba(0,0,0,0.05)' }
                }
            }
        }
    });
}

function pushChartData(val) {
    if (!errorChart) return;
    const data = errorChart.data.datasets[0].data;
    data.push(val);
    if (data.length > 30) data.shift();
    errorChart.update('none');
}

// =============================================================================
// EVENT LISTENERS & INITIALIZATION
// =============================================================================

function setupEventListeners() {
    // Locate me button
    DOM.locateMeBtn.addEventListener("click", () => detectRealUserGpsLocation(true));

    // Recenter buttons (both main pill and floating FAB)
    DOM.recenterMainBtn.addEventListener("click", triggerRecenter);
    DOM.recenterBtn.addEventListener("click", triggerRecenter);

    // Search input
    DOM.destinationInput.addEventListener("input", (e) => {
        const q = e.target.value.trim();
        DOM.clearSearchBtn.classList.toggle("hidden", !q);
        clearTimeout(searchDebounceTimer);
        searchDebounceTimer = setTimeout(() => performSearch(q), 250);
    });

    DOM.destinationInput.addEventListener("focus", () => {
        if (!DOM.destinationInput.value) performSearch("");
    });

    DOM.clearSearchBtn.addEventListener("click", () => {
        DOM.destinationInput.value = "";
        DOM.clearSearchBtn.classList.add("hidden");
        DOM.searchSuggestions.classList.add("hidden");
    });

    // POI Chips
    DOM.categoryChips.forEach(chip => {
        chip.addEventListener("click", () => {
            DOM.categoryChips.forEach(c => c.classList.remove("active"));
            chip.classList.add("active");
            const q = chip.getAttribute("data-query");
            if (q === "current") {
                detectRealUserGpsLocation(true);
            } else if (q === "gandhi road") {
                DOM.destinationInput.value = "Gandhi Road";
                selectDestination({
                    name: "Gandhi Road",
                    address: "Anna Nagar / Central Corridor",
                    latitude: appState.currentOrigin.lat + 0.012,
                    longitude: appState.currentOrigin.lon + 0.010
                });
            } else if (q === "tambaram") {
                DOM.destinationInput.value = "Tambaram";
                selectDestination({
                    name: "Tambaram",
                    address: "Tambaram Railway Station, Chennai, Tamil Nadu",
                    latitude: 12.9249,
                    longitude: 80.1000
                });
            } else {
                DOM.destinationInput.value = chip.innerText.trim();
                performSearch(q);
            }
        });
    });

    // 3D Perspective Toggle
    DOM.toggle3dBtn.addEventListener("click", () => set3DView(!appState.is3DView));

    // Compass Reset
    DOM.compassBtn.addEventListener("click", () => {
        triggerRecenter();
    });

    // Sound toggle
    DOM.soundToggleBtn.addEventListener("click", () => {
        appState.voiceEnabled = !appState.voiceEnabled;
        DOM.soundToggleBtn.classList.toggle("active", appState.voiceEnabled);
        if (appState.voiceEnabled) speakGuidance("Voice guidance enabled.");
    });

    // Quick GNSS Outage Trigger FAB
    DOM.quickGnssOutageBtn.addEventListener("click", () => {
        if (appState.gnssStatus === "GNSS_LOST") {
            restoreGnss();
        } else {
            triggerGnssLoss(25.0);
        }
    });

    // Start / Exit Navigation
    DOM.startNavBtn.addEventListener("click", startNavigation);
    DOM.exitNavBtn.addEventListener("click", exitNavigation);

    // Drawer Menu
    DOM.menuBtn.addEventListener("click", () => {
        DOM.sideDrawer.classList.add("open");
        DOM.menuOverlay.classList.remove("hidden");
    });

    const closeDrawer = () => {
        DOM.sideDrawer.classList.remove("open");
        DOM.menuOverlay.classList.add("hidden");
    };

    DOM.closeDrawerBtn.addEventListener("click", closeDrawer);
    DOM.menuOverlay.addEventListener("click", closeDrawer);

    DOM.menuGpsBtn.addEventListener("click", (e) => {
        e.preventDefault();
        closeDrawer();
        detectRealUserGpsLocation(true);
    });

    DOM.menuMetricsBtn.addEventListener("click", (e) => {
        e.preventDefault();
        closeDrawer();
        DOM.metricsModal.classList.remove("hidden");
        if (!errorChart) initErrorChart();
    });

    DOM.driveMetricsBtn.addEventListener("click", () => {
        DOM.metricsModal.classList.remove("hidden");
        if (!errorChart) initErrorChart();
    });

    DOM.closeMetricsBtn.addEventListener("click", () => {
        DOM.metricsModal.classList.add("hidden");
    });

    // Modal Outage Buttons
    DOM.outage15sBtn.addEventListener("click", () => triggerGnssLoss(15.0));
    DOM.outage30sBtn.addEventListener("click", () => triggerGnssLoss(30.0));
    DOM.outageToggleBtn.addEventListener("click", () => {
        if (appState.gnssStatus === "GNSS_LOST") restoreGnss();
        else triggerGnssLoss(60.0);
    });
    DOM.outageRestoreBtn.addEventListener("click", restoreGnss);
}

// =============================================================================
// APP BOOTSTRAP
// =============================================================================

window.addEventListener("DOMContentLoaded", async () => {
    initMap();
    setupEventListeners();

    // Request default route for immediate ready-to-go experience
    await requestRoute(appState.currentOrigin, appState.currentDestination);

    // Prompt real GPS detection
    detectRealUserGpsLocation(false);
});
