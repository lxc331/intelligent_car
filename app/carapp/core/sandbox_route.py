"""Shortest permitted road route with a checked connector from actual pose."""
from dataclasses import dataclass
import math
from carapp.core.road_network import RoadNetwork,RoadNode
from carapp.core.navigation_path import interpolate,swept_clear


@dataclass(frozen=True)
class RoadRoute:
    points: tuple
    nodes: tuple
    edges: tuple
    length: float


def plan_route(project,current,goal,*,blocked=(),max_entry=.25):
    source=project.network;keepout=project.keepout.grid;observed=project.localization.grid
    disabled=set() if project.metadata['narrow_enabled'] else set(project.metadata['narrow_nodes'])
    blocked=set(tuple(edge) for edge in blocked)
    nodes=[]
    for node in source:
        targets=[]
        for target in node.to:
            if node.id in disabled or target in disabled or (node.id,target) in blocked:continue
            if project.metadata['edge_widths'].get('%d:%d'%(node.id,target),0.) < .23:continue
            other=source.node(target);yaw=math.atan2(other.y-node.y,other.x-node.x)
            path=((node.x,node.y,yaw),(other.x,other.y,yaw))
            if swept_clear(keepout,path) and swept_clear(observed,path):targets.append(target)
        nodes.append(RoadNode(node.id,node.x,node.y,node.yaw,tuple(targets),node.name))
    graph=RoadNetwork(nodes)
    candidates=[]
    for first in graph:
        for target in first.to:
            second=graph.node(target);dx,dy=second.x-first.x,second.y-first.y
            length=dx*dx+dy*dy
            t=max(0.,min(1.,((current[0]-first.x)*dx+(current[1]-first.y)*dy)/length)) if length else 0.
            projection=(first.x+t*dx,first.y+t*dy,math.atan2(dy,dx))
            if math.dist(current[:2],projection[:2])>max_entry:continue
            try:ids=graph.shortest_path(target,goal)
            except ValueError:continue
            vertices=[current,projection]+[(graph.node(i).x,graph.node(i).y,0.) for i in ids]
            path=interpolate(vertices)
            if not swept_clear(keepout,path) or not swept_clear(observed,path):continue
            distance=sum(math.dist(a[:2],b[:2]) for a,b in zip(path,path[1:]))
            edges=((first.id,target),)+tuple(zip(ids,ids[1:]))
            candidates.append(RoadRoute(path,(first.id,)+ids,edges,distance))
    if not candidates:
        raise ValueError('当前位姿到目标没有整车可通行的道路路径；请检查配准、道路宽度或封闭路段')
    return min(candidates,key=lambda route:(route.length,route.nodes))
