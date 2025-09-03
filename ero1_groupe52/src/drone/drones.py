from networkx import eulerize, eulerian_circuit, Graph
import random
import numpy as np
from sklearn.cluster import KMeans
from pyproj import Geod
import networkx as nx
from scipy.spatial.distance import euclidean
from concurrent.futures import ThreadPoolExecutor, as_completed

"""
Simule l'inspection aérienne de neige sur le réseau routier de Montréal par plusieurs drones.

Parameters:
    G          : networkx.Graph (graphe routier)
    nb_drones  : int               (nombre de drones)
    centers    : int               (nombre de “centres de neige”)
    radius     : int               (rayon autour de chaque centre, en mètres)

Returns:
    G          : networkx.Graph enrichi du flag 'enneige'
    trajets    : List[List[int]]   (trajectoires, liste de nœuds, par drone)
    longueur   : float             (distance totale parcourue en km, arrondie)
"""
def reconnaissance_drone(G, nb_drones, centers, radius):
    positions = np.array([[data['x'], data['y']] for _, data in G.nodes(data=True)])
    longeur = sum(data['length'] / 1000 for u, v, data in G.edges(data=True))
    print(f"longueur totale du graph = {longeur} km")
    print(nx.is_connected(G))
    nodes = list(G.nodes())
    geod = Geod(ellps="WGS84")

    node_coords = {n: (G.nodes[n]['y'], G.nodes[n]['x']) for n in nodes}

    snow_centers = random.sample(list(node_coords.values()), centers)
    def is_snowy(u, v):
        """Retourne True si l'un des deux nœuds (u ou v) est dans un rayon enneigé."""
        for lat2, lon2 in snow_centers:
            _, _, du = geod.inv(node_coords[u][1], node_coords[u][0], lon2, lat2)
            _, _, dv = geod.inv(node_coords[v][1], node_coords[v][0], lon2, lat2)
            if du <= radius or dv <= radius:
                return True
        return False

    n_clusters = max(50, nb_drones)

    kmeans = KMeans(n_clusters=n_clusters, random_state=42)
    labels = kmeans.fit_predict(positions)

    clusters = [[] for _ in range(n_clusters)]
    for node, lbl in zip(nodes, labels):
        clusters[lbl].append(node)

    cluster_centers = kmeans.cluster_centers_

    def order(centers):
        n = len(centers)
        visited = [0]
        to_visit = set(range(1, n))
        current = 0
        while to_visit:
            next_node = min(to_visit, key=lambda j: np.linalg.norm(centers[current] - centers[j]))
            visited.append(next_node)
            to_visit.remove(next_node)
            current = next_node
        return visited

    cluster_order = order(cluster_centers)
    cluster_per_drone = len(cluster_order) // nb_drones
    remainder = len(cluster_order) % nb_drones

    cluster_assignements = []
    start = 0
    for i in range(nb_drones):
        count = cluster_per_drone + (1 if i < remainder else 0)
        cluster_assignements.append(cluster_order[start:start+count])
        start += count

    print("Zone(s) pour chaque drone :", cluster_assignements, flush=True)

    # Attribution de chaque arête à un cluster ce qui etait pas fait avant
    
    node_to_cluster = {node: lbl for node, lbl in zip(nodes, labels)}
    edge_to_cluster = {}
    edges_in_cluster = {i: [] for i in range(n_clusters)}

    for u, v, data in G.edges(data=True):
        cu = node_to_cluster[u]
        cv = node_to_cluster[v]
        if cu == cv:
            chosen = cu
        else:
            x_u, y_u = G.nodes[u]['x'], G.nodes[u]['y']
            x_v, y_v = G.nodes[v]['x'], G.nodes[v]['y']
            mid_x, mid_y = (x_u + x_v) / 2, (y_u + y_v) / 2
            dist_u = (mid_x - cluster_centers[cu][0])**2 + (mid_y - cluster_centers[cu][1])**2
            dist_v = (mid_x - cluster_centers[cv][0])**2 + (mid_y - cluster_centers[cv][1])**2
            chosen = cu if dist_u <= dist_v else cv
        key = (u, v) if u < v else (v, u)
        edge_to_cluster[key] = chosen
        edges_in_cluster[chosen].append(key)

    subgraphs_per_cluster = {}
    for i in range(n_clusters):
        SG = nx.Graph()
        for (u, v) in edges_in_cluster[i]:
            if G.has_edge(u, v):
                for key_attr, attrdict in G[u][v].items():
                    SG.add_edge(u, v, **attrdict)
                    break
            else:
                for key_attr, attrdict in G[v][u].items():
                    SG.add_edge(v, u, **attrdict)
                    break
        subgraphs_per_cluster[i] = SG

    trajets = [None] * nb_drones
    longueurs = [0.0] * nb_drones

    def _compute(drone_id):
        trajet = []
        dist_km = 0.0
        last_node = None
        nb = 0
        for cluster_id in cluster_assignements[drone_id]:
            nb += 1
            SG_i = subgraphs_per_cluster[cluster_id]
            if SG_i.number_of_edges() == 0:
                continue
            components = [SG_i.subgraph(c).copy() for c in nx.connected_components(SG_i)]
            for comp in components:
                cx, cy = cluster_centers[cluster_id]
                start_node = min(
                    comp.nodes,
                    key=lambda n: (G.nodes[n]['x'] - cx)**2 + (G.nodes[n]['y'] - cy)**2
                )
                if last_node is not None:
                    chemin = nx.shortest_path(G, last_node, start_node, weight='length')
                    trajet.extend(chemin if not trajet else chemin[1:])
                    for u, v in zip(chemin, chemin[1:]):
                        if G.has_edge(u, v):
                            for key_attr, attrdict in G[u][v].items():
                                dist_km += attrdict.get('length', 0) / 1000.0
                                break
                        else:
                            for key_attr, attrdict in G[v][u].items():
                                dist_km += attrdict.get('length', 0) / 1000.0
                                break
                comp_simple = Graph()
                comp_simple.add_edges_from(comp.edges(data=True))
                comp_euler = eulerize(comp_simple)
                circuit = list(eulerian_circuit(comp_euler, source=start_node))
                tour = [start_node] + [v for (_, v) in circuit]
                visited_edges = set()
                for u, v in circuit:
                    if (u, v) in visited_edges or (v, u) in visited_edges:
                        continue
                    if G.has_edge(u, v):
                        for key_attr, attrdict in G[u][v].items():
                            attrdict['enneige'] = is_snowy(u, v)
                            dist_km += attrdict.get('length', 0) / 1000.0
                            break
                    else:
                        for key_attr, attrdict in G[v][u].items():
                            attrdict['enneige'] = is_snowy(u, v)
                            dist_km += attrdict.get('length', 0) / 1000.0
                            break
                    visited_edges.add((u, v))
                trajet.extend(tour if not trajet else tour[1:])
                last_node = tour[-1]
            print(f"[Drone {drone_id}] a terminé la zone : {cluster_id}     {nb} / {max(50, nb_drones)}", flush=True)
        print(f"[Drone {drone_id}] terminé : {len(trajet)} nœuds parcourus, {dist_km:.2f} km parcourus", flush=True)
        return drone_id, trajet, dist_km

    with ThreadPoolExecutor(max_workers=nb_drones) as executor:
        futures = [executor.submit(_compute, i) for i in range(nb_drones)]
        for fut in as_completed(futures):
            did, tr, ln = fut.result()
            trajets[did] = tr
            longueurs[did] = ln

    total = round(sum(longueurs), 3)
    return G, trajets, total
