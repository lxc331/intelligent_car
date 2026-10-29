"""The user's measured three-by-three sandbox roads; no invented alignment."""
import math
from carapp.core.road_network import RoadNetwork, RoadNode


def standard_layout(road_map, registration):
    if any(abs(v)>1e-8 for v in road_map.origin):raise ValueError('手绘模板必须使用原始零原点地图')
    if abs(road_map.width*road_map.resolution-2.97)>.02 or abs(road_map.height*road_map.resolution-5.4)>.02:
        raise ValueError('九路口模板对应当前 2.97×5.40 m 手绘沙盘，请导入 rawMap.yaml')
    xs,ys = (.25,1.65,2.845),(.27,3.125,5.13)
    nodes,widths = [],{}
    for row,y in enumerate(ys):
        for col,x in enumerate(xs):
            node_id=row*3+col
            targets=[]
            for dc,dr in ((-1,0),(1,0),(0,-1),(0,1)):
                r,c=row+dr,col+dc
                if 0<=r<3 and 0<=c<3:
                    other=r*3+c;targets.append(other)
                    widths['%d:%d'%(node_id,other)] = (.50,.52,.25)[col] if dr else (.52,.51,.52)[row]
            wx,wy=registration.forward(x,y)
            nodes.append(RoadNode(node_id,wx,wy,None,tuple(targets),'路口%d'%node_id))
    return RoadNetwork(nodes),widths


def build_keepout(localization, road_map, registration):
    """Full localization extent; white roads only, black everywhere else."""
    grid=road_map.grid; data=grid['data']; width,height=localization.width,localization.height
    ox,oy,angle=localization.origin;c,s=math.cos(angle),math.sin(angle)
    rx,ry,ra=road_map.origin;rc,rs=math.cos(ra),math.sin(ra)
    output=bytearray(width*height)
    for row in range(height):
        for col in range(width):
            dx,dy=(col+.5)*localization.resolution,(row+.5)*localization.resolution
            x,y=registration.inverse(ox+c*dx-s*dy,oy+s*dx+c*dy)
            dx,dy=x-rx,y-ry
            gx=math.floor((rc*dx+rs*dy)/road_map.resolution)
            gy=math.floor((-rs*dx+rc*dy)/road_map.resolution)
            if 0<=gx<road_map.width and 0<=gy<road_map.height and data[gy*road_map.width+gx]==0:
                output[(height-1-row)*width+col]=254
    header=('P5\n%d %d\n255\n'%(width,height)).encode()
    return header+output
