"""
geocoding_service.py
====================
Geocoding and Landmark Search Service for MapX.
Integrates online Nominatim / Photon queries with comprehensive preset landmarks
focused on Indian Space Research Organisation (ISRO) centres, tech parks, and urban hubs.
"""

from typing import List, Dict, Any
import httpx
import logging

logger = logging.getLogger(__name__)

# Curated high-precision landmarks for seamless immediate access
PRESET_LANDMARKS = [
    {
        "id": "gandhi_road",
        "name": "Gandhi Road (Central)",
        "address": "Gandhi Road, Anna Nagar / T. Nagar, Chennai, Tamil Nadu",
        "category": "Main Road",
        "latitude": 13.0850,
        "longitude": 80.2780,
        "icon": "🛣️"
    },
    {
        "id": "isro_hq",
        "name": "ISRO Headquarters (Antariksh Bhavan)",
        "address": "New BEL Road, Bengaluru, Karnataka 560094",
        "category": "Space / Government",
        "latitude": 13.0335,
        "longitude": 77.5640,
        "icon": "🚀"
    },
    {
        "id": "ursc_bangalore",
        "name": "U R Rao Satellite Centre (URSC)",
        "address": "HAL Airport Road, Vimanapura, Bengaluru, Karnataka 560017",
        "category": "ISRO Center",
        "latitude": 12.9592,
        "longitude": 77.6668,
        "icon": "🛰️"
    },
    {
        "id": "isro_telemetry",
        "name": "ISTRAC Ground Station",
        "address": "Peenya Industrial Area, Bengaluru, Karnataka 560058",
        "category": "Space Tracking",
        "latitude": 13.0286,
        "longitude": 77.5184,
        "icon": "📡"
    },
    {
        "id": "vssc_thiruvananthapuram",
        "name": "Vikram Sarabhai Space Centre (VSSC)",
        "address": "Thumba, Thiruvananthapuram, Kerala 695022",
        "category": "Space Center",
        "latitude": 8.5312,
        "longitude": 76.8682,
        "icon": "🚀"
    },
    {
        "id": "sadh_shar",
        "name": "Satish Dhawan Space Centre (SDSC-SHAR)",
        "address": "Sriharikota, Andhra Pradesh 524124",
        "category": "Launch Center",
        "latitude": 13.7200,
        "longitude": 80.2300,
        "icon": "🎯"
    },
    {
        "id": "iisc_bangalore",
        "name": "Indian Institute of Science (IISc)",
        "address": "CV Raman Rd, Bengaluru, Karnataka 560012",
        "category": "Research Institute",
        "latitude": 13.0219,
        "longitude": 77.5671,
        "icon": "🎓"
    },
    {
        "id": "kempegowda_airport",
        "name": "Kempegowda International Airport (BLR)",
        "address": "Devanahalli, Bengaluru, Karnataka 560300",
        "category": "Airport",
        "latitude": 13.1986,
        "longitude": 77.7066,
        "icon": "✈️"
    },
    {
        "id": "manyata_tech_park",
        "name": "Manyata Embassy Business Park",
        "address": "Outer Ring Road, Nagavara, Bengaluru 560045",
        "category": "Tech Park",
        "latitude": 13.0487,
        "longitude": 77.6200,
        "icon": "🏢"
    },
    {
        "id": "electronic_city",
        "name": "Electronic City Phase 1",
        "address": "Hosur Road, Bengaluru, Karnataka 560100",
        "category": "IT Corridor",
        "latitude": 12.8452,
        "longitude": 77.6602,
        "icon": "💻"
    },
    {
        "id": "cubbon_park",
        "name": "Cubbon Park / MG Road",
        "address": "Kasturba Road, Bengaluru, Karnataka 560001",
        "category": "City Center",
        "latitude": 12.9763,
        "longitude": 77.5929,
        "icon": "🌳"
    }
]


class GeocodingService:
    """Provides instant location search and geocoding."""

    async def search(self, query: str, limit: int = 6) -> List[Dict[str, Any]]:
        """Search query matching presets or querying Nominatim."""
        query_clean = query.strip().lower()
        if not query_clean:
            return PRESET_LANDMARKS[:limit]

        # 1. Match in presets
        matched = []
        for lm in PRESET_LANDMARKS:
            if query_clean in lm["name"].lower() or query_clean in lm["address"].lower() or query_clean in lm["category"].lower():
                matched.append(lm)

        if len(matched) >= limit:
            return matched[:limit]

        # 2. Query Nominatim OpenStreetMap Geocoder with timeout
        try:
            url = f"https://nominatim.openstreetmap.org/search?q={query}&format=json&addressdetails=1&limit={limit}"
            async with httpx.AsyncClient(timeout=3.0) as client:
                response = await client.get(url, headers={"User-Agent": "MapX-Navigation-Client/1.0"})
                if response.status_code == 200:
                    results = response.json()
                    for item in results:
                        matched.append({
                            "id": f"osm_{item.get('osm_id', item.get('place_id'))}",
                            "name": item.get("name") or item.get("display_name", "").split(",")[0],
                            "address": item.get("display_name", ""),
                            "category": item.get("type", "Location").replace("_", " ").title(),
                            "latitude": float(item["lat"]),
                            "longitude": float(item["lon"]),
                            "icon": "📍"
                        })
        except Exception as e:
            logger.warning(f"Online geocoding lookup failed: {e}")

        # Deduplicate and return
        unique_results = []
        seen_coords = set()
        for res in matched:
            coord_key = (round(res["latitude"], 4), round(res["longitude"], 4))
            if coord_key not in seen_coords:
                seen_coords.add(coord_key)
                unique_results.append(res)

        return unique_results[:limit] if unique_results else PRESET_LANDMARKS[:limit]

    def get_saved_places(self) -> List[Dict[str, Any]]:
        """Return curated saved destinations."""
        return PRESET_LANDMARKS
