import argparse
import osmnx as ox
import folium
from folium.plugins import TimestampedGeoJson
import datetime
import networkx as nx
from collections import defaultdict
import time

from drone.drones import reconnaissance_drone

def generate_graph():
    try:
        G = ox.load_graphml("montreal.graphml")
    except FileNotFoundError:
        G = ox.graph_from_place(
            "Montréal, Québec, Canada",
            network_type="drive",
            simplify=True
        )
        ox.save_graphml(G, "montreal.graphml")
    return G

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--drones", type=int, default=1)
    parser.add_argument("--centers", type=int, default=5)
    parser.add_argument("--radius", type=int, default=500)
    args = parser.parse_args()

    G_drive = generate_graph()
    G_undirected = ox.convert.to_undirected(G_drive)

    start_exec = time.time()
    G_result, trajets, longueurs = reconnaissance_drone(
        G_undirected,
        nb_drones=args.drones,
        centers=args.centers,
        radius=args.radius
    )
    duration_min = (time.time() - start_exec) / 60.0

    ox.save_graphml(G_result, "snow.graphml")

    nb_snowy = sum(1 for u, v, data in G_result.edges(data=True) if data.get("enneige"))
    print(f"[DEBUG] nombre d'arêtes enneigées détectées par reconnaissance_drone : {nb_snowy}")

    for u, v, data in G_result.edges(data=True):
        data.setdefault('enneige', False)

    nodes_gdf = ox.graph_to_gdfs(G_result, nodes=True, edges=False)
    centre = [nodes_gdf['y'].mean(), nodes_gdf['x'].mean()]

    m = folium.Map(location=centre, zoom_start=12, tiles="OpenStreetMap")

    edges_gdf = ox.graph_to_gdfs(G_result, nodes=False, edges=True)
    for _, row in edges_gdf.iterrows():
        if row['enneige']:
            geom = row.geometry
            lines = [geom] if geom.geom_type == "LineString" else list(geom)
            for line in lines:
                coords = [[lat, lon] for lon, lat in line.coords]
                folium.PolyLine(
                    locations=coords,
                    color="hsl(0,100%,75%)",
                    weight=6,
                    opacity=0.6,
                    smooth_factor=1
                ).add_to(m)

    segment_counts = defaultdict(int)
    for path in trajets:
        for u, v in zip(path, path[1:]):
            key = tuple(sorted((u, v)))
            segment_counts[key] += 1

    start_time = datetime.datetime.now()
    dt_per_m = datetime.timedelta(milliseconds=1)
    traj_features = []
    snow_features = []
    epsilon = datetime.timedelta(seconds=1)

    for path in trajets:
        t = start_time
        for u, v in zip(path, path[1:]):
            y1, x1 = G_result.nodes[u]['y'], G_result.nodes[u]['x']
            y2, x2 = G_result.nodes[v]['y'], G_result.nodes[v]['x']

            length_m = nx.shortest_path_length(G_result, u, v, weight='length')
            t_end = t + dt_per_m * length_m

            cnt = segment_counts[tuple(sorted((u, v)))]
            lightness = max(20, 80 - (cnt - 1) * 15)
            blue = f"hsl(240,100%,{lightness}%)"

            traj_features.append({
                "type": "Feature",
                "geometry": {
                    "type": "LineString",
                    "coordinates": [[x1, y1], [x2, y2]]
                },
                "properties": {
                    "times": [t.isoformat(), t_end.isoformat()],
                    "style": {"color": blue, "weight": 3, "opacity": 1}
                }
            })

            data_dict = G_result.get_edge_data(u, v) or {}
            snowy = any(attr.get('enneige', False) for attr in data_dict.values())
            if snowy:
                snow_features.append({
                    "type": "Feature",
                    "geometry": {
                        "type": "LineString",
                        "coordinates": [[x1, y1], [x2, y2]]
                    },
                    "properties": {
                        "times": [t_end.isoformat(), (t_end + epsilon).isoformat()],
                        "style": {"color": "red", "weight": 6, "opacity": 0.8}
                    }
                })

            t = t_end

    all_features = traj_features + snow_features

    TimestampedGeoJson(
        {
            "type": "FeatureCollection",
            "features": all_features
        },
        period="PT1S",
        add_last_point=False,
        auto_play=True,
        loop=False
    ).add_to(m)

    m.save("montreal_drones.html")
    print(f"Carte enregistrée : montreal_drones.html")
    print(f"Temps d'exécution : {duration_min:.2f} minutes")
