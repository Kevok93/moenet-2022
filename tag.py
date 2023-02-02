import json
import numpy as np

tagf = open('tags.json')

data = json.load(tagf)

tagpos = [0]*9
rot180 = [0]*9

for tag in data["tags"]:
    posvec = []
    for pos in tag['pose']['translation'].values():
        posvec += [float(pos)]
    posvec = np.array(posvec)
    tagpos[tag['ID']] = posvec
    if tag['pose']['rotation']['quaternion']['Z']:
        rot180[tag['ID']] = 1