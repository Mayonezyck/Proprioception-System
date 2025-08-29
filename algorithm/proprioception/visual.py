import networkx as nx
import matplotlib.pyplot as plt

# Create directed graph
G = nx.DiGraph()

# Define nodes
sensors = ["Muscle Spindle", "Golgi Tendon Organ", "Joint Capsule Mechanoreceptors"]
modalities = ["Vision", "Tactile"]
processing = [
    "Spinal Cord Reflex", "Dorsal Columns",
    "Dorsal Spinocerebellar Tract", "Ventral Spinocerebellar Tract",
    "VPL Thalamus", "S1 (Somatosensory Cortex)",
    "M1 (Motor Cortex)", "Cerebellum",
    "Multimodal Integration", "Cerebrum", "Motor Decision", "Motor Output"
]

# Add all nodes
G.add_nodes_from(sensors + modalities + processing)

# Reflex pathway: all sensors to spinal cord reflex
for s in sensors:
    G.add_edge(s, "Spinal Cord Reflex")

# Conscious pathway: all sensors to dorsal columns
for s in sensors:
    G.add_edge(s, "Dorsal Columns")

# Spinocerebellar pathways: only muscle spindle and GTO
for s in ["Muscle Spindle", "Golgi Tendon Organ"]:
    G.add_edge(s, "Dorsal Spinocerebellar Tract")
    G.add_edge(s, "Ventral Spinocerebellar Tract")

# Modalities to integration
for m in modalities:
    G.add_edge(m, "Multimodal Integration")

# Spinal reflex to output
G.add_edge("Spinal Cord Reflex", "Motor Output")

# Dorsal columns conscious path
G.add_edge("Dorsal Columns", "VPL Thalamus")
G.add_edge("VPL Thalamus", "S1 (Somatosensory Cortex)")
G.add_edge("S1 (Somatosensory Cortex)", "M1 (Motor Cortex)")

# Spinocerebellar to cerebellum
G.add_edge("Dorsal Spinocerebellar Tract", "Cerebellum")
G.add_edge("Ventral Spinocerebellar Tract", "Cerebellum")

# Cortical, cerebellar, integration to cerebrum
G.add_edge("M1 (Motor Cortex)", "Cerebrum")
G.add_edge("S1 (Somatosensory Cortex)", "Cerebrum")
G.add_edge("Cerebellum", "Cerebrum")
G.add_edge("Multimodal Integration", "Cerebrum")

# Cerebrum to decision and output
G.add_edge("Cerebrum", "Motor Decision")
G.add_edge("Motor Decision", "Motor Output")

# Manual positions
pos = {
    "Vision": (4.5, 2.5),
    "Muscle Spindle": (0, 4),
    "Golgi Tendon Organ": (0, 3.5),
    "Joint Capsule Mechanoreceptors": (2, 3.5),
    "Tactile": (4, 2.5),
    "Spinal Cord Reflex": (2, 4),
    "Dorsal Columns": (2, 3),
    "Dorsal Spinocerebellar Tract": (2, 2),
    "Ventral Spinocerebellar Tract": (2, 1),
    "VPL Thalamus": (4, 3),
    "S1 (Somatosensory Cortex)": (6, 3),
    "M1 (Motor Cortex)": (7, 2.8),
    "Cerebellum": (6, 1),
    "Multimodal Integration": (4, 1),
    "Cerebrum": (7, 2),
    "Motor Decision": (10, 2.5),
    "Motor Output": (12, 2.5)
}

# Draw the DAG
plt.figure(figsize=(14, 8))
nx.draw(G, pos, with_labels=True, arrows=True, node_size=2000, font_size=10)
plt.axis('off')
plt.show()

