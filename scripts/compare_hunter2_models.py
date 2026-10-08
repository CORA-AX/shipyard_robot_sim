#!/usr/bin/env python3
"""Offline, non-mutating Hunter audit. Python 3.10+ standard library only.

The restricted Xacro reader supports exactly includes, properties, simple macros,
${numeric arithmetic/parameter}, and $(find hunter_description) in these inputs.
It is NOT a general Xacro implementation and rejects unsupported constructs.
Outputs (optional) are restricted to docs/; canonical assets are never written.
"""
import argparse
import ast
from collections import Counter
import copy
import difflib
from functools import lru_cache
import hashlib
import json
import math
import operator
from pathlib import Path
import re
import struct
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
AGX = ROOT / 'third_party/ugv_gazebo_sim/hunter/hunter2_base'
LCAS = ROOT / 'third_party/hunter_robot/hunter_description'
X = '{http://www.ros.org/wiki/xacro}'
IDENTITY = [[float(i == j) for j in range(4)] for i in range(4)]


def require(ok, message):
    if not ok:
        raise ValueError(message)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def nums(s):
    v = list(map(float, s.split()))
    require(all(math.isfinite(x) for x in v), 'Non-finite coordinate')
    return v


def mm(a, b):
    return [[sum(a[i][k]*b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def point(m, v, w=1):
    return [sum(m[i][k]*v[k] for k in range(3))+m[i][3]*w for i in range(3)]


def spin(axis, q):
    norm = math.sqrt(sum(a*a for a in axis))
    require(norm > 0, 'Zero moving-joint axis')
    x,y,z = [a/norm for a in axis]
    c,s = math.cos(q),math.sin(q)
    t = 1-c
    return [[t*x*x+c,t*x*y-s*z,t*x*z+s*y,0],
            [t*x*y+s*z,t*y*y+c,t*y*z-s*x,0],
            [t*x*z-s*y,t*y*z+s*x,t*z*z+c,0],[0,0,0,1]]


def pose(xyz=(0,0,0), rpy=(0,0,0)):
    r,p,y = rpy
    m = mm(mm(spin((0,0,1),y),spin((0,1,0),p)),spin((1,0,0),r))
    for i in range(3):
        m[i][3] = xyz[i]
    return m


def origin(element):
    o = element.find('origin')
    return {'xyz': nums(o.get('xyz','0 0 0')) if o is not None else [0,0,0],
            'rpy': nums(o.get('rpy','0 0 0')) if o is not None else [0,0,0]}


def safe_expr(expression, env):
    ops = {ast.Add: operator.add,ast.Sub: operator.sub,ast.Mult: operator.mul,ast.Div: operator.truediv}
    def visit(n):
        if isinstance(n,ast.Constant) and type(n.value) in (int,float): return n.value
        if isinstance(n,ast.Name) and n.id in env: return env[n.id]
        if isinstance(n,ast.BinOp) and type(n.op) in ops: return ops[type(n.op)](visit(n.left),visit(n.right))
        if isinstance(n,ast.UnaryOp) and isinstance(n.op,(ast.USub,ast.UAdd)):
            return (-1 if isinstance(n.op,ast.USub) else 1)*visit(n.operand)
        raise ValueError(f'Unsupported Xacro expression: {expression}')
    return visit(ast.parse(expression,mode='eval').body)


class RestrictedXacro:
    def __init__(self):
        self.env = {'pi': math.pi}
        self.macros = {}
        self.files = set()

    def subst(self, text, env):
        text = text.replace('$(find hunter_description)', str(LCAS.relative_to(ROOT)))
        require('$(' not in text, f'Unsupported substitution: {text}')
        return re.sub(r'\$\{([^}]+)\}',lambda m: str(safe_expr(m[1],env)),text)

    def expand(self, elements, directory, env, depth=0):
        require(depth < 30,'Xacro recursion depth')
        output = []
        for e in elements:
            if e.tag == X+'include':
                path = (directory/self.subst(e.get('filename'),env)).resolve()
                require(path.is_relative_to(LCAS),'Include outside reviewed package')
                self.files.add(path)
                output += self.expand(list(ET.parse(path).getroot()),path.parent,env,depth+1)
            elif e.tag == X+'property':
                value = self.subst(e.get('value'),env)
                env[e.get('name')] = float(value)
            elif e.tag == X+'macro':
                self.macros[e.get('name')] = (e,directory)
            elif e.tag.startswith(X):
                name = e.tag[len(X):]
                require(name in self.macros,f'Unsupported Xacro tag: {name}')
                macro, macro_dir = self.macros[name]
                params = macro.get('params','').split()
                require(set(params)==set(e.attrib),f'Macro argument mismatch: {name}')
                local = dict(env)
                local.update({p:self.subst(e.get(p),env) for p in params})
                output += self.expand(list(macro),macro_dir,local,depth+1)
            else:
                n = ET.Element(e.tag,{k:self.subst(v,env) for k,v in e.attrib.items()})
                n.text = self.subst(e.text,env) if e.text and e.text.strip() else None
                n.extend(self.expand(list(e),directory,dict(env),depth+1))
                output.append(n)
        return output

    def read(self,path):
        self.files.add(path)
        r = ET.parse(path).getroot()
        out = ET.Element('robot',r.attrib)
        out.extend(self.expand(list(r),path.parent,self.env))
        return out


def mesh_path(uri, urdf):
    if uri.startswith('package://hunter2_base/'):
        return AGX / uri.removeprefix('package://hunter2_base/')
    if uri.startswith('file://third_party/'):
        return ROOT / uri.removeprefix('file://')
    require('://' not in uri, f'Unsupported mesh URI: {uri}')
    return urdf.parent / uri


@lru_cache(maxsize=None)
def mesh_triangles(path):
    """STL or the reviewed COLLADA triangle subset; no silent scene transforms."""
    if path.suffix.lower()=='.stl':
        b=path.read_bytes()
        require(len(b)>=84 and len(b)==84+50*struct.unpack_from('<I',b,80)[0],f'Invalid STL: {path}')
        return [tuple(tuple(t[i:i+3]) for i in (3,6,9)) for t in struct.iter_unpack('<12fH',b[84:])]
    require(path.suffix.lower()=='.dae',f'Unsupported mesh: {path}')
    root=ET.parse(path).getroot()
    for e in root.iter(): e.tag=e.tag.split('}')[-1]
    require(root.findtext('asset/up_axis')=='Z_UP','Unexpected DAE up-axis')
    scale=float(root.find('asset/unit').get('meter'))
    geometries={}
    for g in root.findall('library_geometries/geometry'):
        mesh=g.find('mesh'); sources={}; vertices={}
        require(not any(mesh.findall(tag) for tag in ('polylist','polygons','trifans','tristrips')),'Unsupported DAE primitive')
        for s in mesh.findall('source'):
            a=s.find('technique_common/accessor')
            raw=nums(s.findtext('float_array')); stride=int(a.get('stride','1')); offset=int(a.get('offset','0'))
            sources[s.get('id')]=[raw[offset+i*stride:offset+i*stride+3] for i in range(int(a.get('count')))]
        for v in mesh.findall('vertices'):
            vertices[v.get('id')]=v.find("input[@semantic='POSITION']").get('source')[1:]
        tris=[]
        for primitive in mesh.findall('triangles'):
            inputs=primitive.findall('input'); stride=max(int(i.get('offset','0')) for i in inputs)+1
            vertex=primitive.find("input[@semantic='VERTEX']")
            require(vertex is not None,'DAE triangle lacks VERTEX input')
            offset=int(vertex.get('offset','0')); positions=sources[vertices[vertex.get('source')[1:]]]
            idx=list(map(int,primitive.findtext('p').split()))
            require(len(idx)==int(primitive.get('count'))*3*stride,'DAE index count mismatch')
            tris += [tuple(tuple(positions[idx[k+j*stride+offset]]) for j in range(3)) for k in range(0,len(idx),3*stride)]
        geometries[g.get('id')]=tris
    scene_id=root.find('scene/instance_visual_scene').get('url')[1:]
    scene=root.find(f"library_visual_scenes/visual_scene[@id='{scene_id}']")
    triangles=[]
    def walk(node,parent):
        m=parent
        for child in node:
            if child.tag=='matrix':
                # Reviewed exports all use identity; reject other matrices rather than guess layout.
                require(nums(child.text)==sum(IDENTITY,[]),'Nonidentity DAE matrix needs a general COLLADA loader')
            elif child.tag in ('translate','rotate','scale','lookat','skew','instance_node'):
                raise ValueError('Unsupported DAE scene transform/instance')
        for inst in node.findall('instance_geometry'):
            triangles.extend(tuple(tuple(v*scale for v in point(m,p)) for p in t) for t in geometries[inst.get('url')[1:]])
        for child in node.findall('node'): walk(child,m)
    for n in scene.findall('node'): walk(n,IDENTITY)
    require(triangles,f'No referenced DAE triangles: {path}')
    return triangles


def bounds(triangles):
    return {'min':[min(p[i] for t in triangles for p in t) for i in range(3)],
            'max':[max(p[i] for t in triangles for p in t) for i in range(3)]}


def topology(root):
    links={e.get('name'):e for e in root.findall('link')}
    joints={e.get('name'):e for e in root.findall('joint')}
    require(len(links)==len(root.findall('link')) and len(joints)==len(root.findall('joint')),'Duplicate names')
    children=[j.find('child').get('link') for j in joints.values()]
    require(len(set(children))==len(children),'Multiple parents')
    require(set(links)-set(children)=={'base_link'},'Expected base_link root')
    for j in joints.values():
        require(j.find('parent').get('link') in links and j.find('child').get('link') in links,'Invalid joint references')
    def frames(q=None):
        q=q or {}; result={'base_link':IDENTITY}; joint_frames={}; pending=dict(joints)
        while pending:
            count=len(pending)
            for name,j in list(pending.items()):
                parent=j.find('parent').get('link')
                if parent not in result: continue
                m=mm(result[parent],pose(**origin(j))); joint_frames[name]=m
                if j.get('type')!='fixed':
                    m=mm(m,spin(nums(j.find('axis').get('xyz')),q.get(name,0)))
                result[j.find('child').get('link')]=m
                del pending[name]
            require(len(pending)<count,'Disconnected/cyclic joint graph')
        return result,joint_frames
    return links,joints,frames


def roles(lcas):
    return dict(steer_left='front_left_steering_joint' if lcas else 'front_steer_left_joint',
                steer_right='front_right_steering_joint' if lcas else 'front_steer_right_joint',
                front_left='front_left_wheel_joint',front_right='front_right_wheel_joint',
                rear_left='rear_left_wheel_joint' if lcas else 'left_rear_joint',
                rear_right='rear_right_wheel_joint' if lcas else 'right_rear_joint')


def inspect_model(root,path,lcas=False):
    links,joints,frames=topology(root); lf,jf=frames(); role=roles(lcas)
    result={'file':str(path.relative_to(ROOT)),'links':{},'joints':{},'joint_roles':role,
            'counts':{'links':len(links),'joints':len(joints)},'total_mass_kg':0,'mesh_inventory':{}}
    for name,j in joints.items():
        axis=nums(j.find('axis').get('xyz')) if j.find('axis') is not None else [0,0,0]
        result['joints'][name]={'type':j.get('type'),'parent':j.find('parent').get('link'),'child':j.find('child').get('link'),
            'origin':origin(j),'axis_local':axis,'axis_base_zero':point(jf[name],axis,0),'position_base':point(jf[name],[0,0,0]),
            'limit':dict(j.find('limit').attrib) if j.find('limit') is not None else None,
            'dynamics':dict(j.find('dynamics').attrib) if j.find('dynamics') is not None else None,
            'mimic':dict(j.find('mimic').attrib) if j.find('mimic') is not None else None}
    for name,link in links.items():
        inertial=link.find('inertial'); data={'inertial':None,'visual':[],'collision':[]}
        if inertial is not None:
            mass=float(inertial.find('mass').get('value')); result['total_mass_kg']+=mass
            data['inertial']={'mass':mass,'origin':origin(inertial),'tensor':{k:float(v) for k,v in inertial.find('inertia').attrib.items()}}
        for kind in ('visual','collision'):
            for e in link.findall(kind):
                shape=list(e.find('geometry'))[0]
                entry={'origin':origin(e),'geometry_type':shape.tag,'geometry':dict(shape.attrib),
                       'material_xml':ET.tostring(e.find('material'),encoding='unicode').strip() if e.find('material') is not None else None}
                if shape.tag=='mesh':
                    f=mesh_path(shape.get('filename'),path).resolve()
                    require(f.is_file(),f'Missing mesh: {f}')
                    rel=str(f.relative_to(ROOT)); entry['resolved_mesh']=rel
                    triangles=mesh_triangles(f)
                    result['mesh_inventory'][rel]={'sha256':sha(f),'triangles':len(triangles),'bounds':bounds(triangles)}
                data[kind].append(entry)
        result['links'][name]=data
    pos={r:result['joints'][n]['position_base'] for r,n in role.items()}
    result['geometry']={'left_wheelbase':pos['front_left'][0]-pos['rear_left'][0],
        'right_wheelbase':pos['front_right'][0]-pos['rear_right'][0],
        'front_track':pos['front_left'][1]-pos['front_right'][1], 'rear_track':pos['rear_left'][1]-pos['rear_right'][1],
        'wheel_positions_base':{k:v for k,v in pos.items() if not k.startswith('steer')}}
    probes={}
    for side in ('left','right'):
        sj,wj=role['steer_'+side],role['front_'+side]; samples=[]
        for angle in (-0.3,0,0.3):
            _,qframes=frames({sj:angle})
            a=point(qframes[wj],result['joints'][wj]['axis_local'],0)
            d=[a[1],-a[0],0]
            if d[0]<0: d=[-v for v in d]
            lim=result['joints'][sj]['limit']
            samples.append({'input_rad':angle,'heading_rad':math.atan2(d[1],d[0]),'axle_base':a,
                            'within_urdf_limit':float(lim['lower'])<=angle<=float(lim['upper'])})
        probes[side]=samples
    result['steering_probe_independent_not_mimic_applied']=probes
    result['wheel_mesh_radius']={}
    for r in ('front_left','front_right','rear_left','rear_right'):
        j=result['joints'][role[r]]; link=result['links'][j['child']]
        shape=link['visual'][0]; triangles=mesh_triangles(ROOT/shape['resolved_mesh'])
        scale=nums(shape['geometry'].get('scale','1 1 1')); m=pose(**shape['origin'])
        pts=[point(m,[p[i]*scale[i] for i in range(3)]) for t in triangles for p in t]
        b={'min':[min(p[i] for p in pts) for i in range(3)],'max':[max(p[i] for p in pts) for i in range(3)]}
        axis=j['axis_local']; n=math.sqrt(sum(v*v for v in axis)); a=[v/n for v in axis]
        radius=max(math.sqrt(max(0,sum(v*v for v in p)-sum(p[i]*a[i] for i in range(3))**2)) for p in pts)
        result['wheel_mesh_radius'][r]={'aabb_xy_half_extent':[(b['max'][i]-b['min'][i])/2 for i in (0,1)],
                                      'max_distance_from_joint_axis':radius,'bounds_link':b}
    result['overlap']={}
    def triangles_in_base(name,kind):
        out=Counter()
        for shape in result['links'][name][kind]:
            if shape['geometry_type']!='mesh': continue
            m=mm(lf[name],pose(**shape['origin'])); scale=nums(shape['geometry'].get('scale','1 1 1'))
            for t in mesh_triangles(ROOT/shape['resolved_mesh']):
                out[tuple(sorted(tuple(round(v,6) for v in point(m,[p[i]*scale[i] for i in range(3)])) for p in t))]+=1
        return out
    for side in ('left','right'):
        s=result['joints'][role['steer_'+side]]['child']; w=result['joints'][role['front_'+side]]['child']
        result['overlap'][side]={}
        for kind in ('visual','collision'):
            a,b=triangles_in_base(s,kind),triangles_in_base(w,kind)
            result['overlap'][side][kind]={'steering_mesh_triangles':sum(a.values()),'wheel_mesh_triangles':sum(b.values()),
                                         'shared_triangles_rounded_1um':sum((a&b).values()),
                                         'steering_shapes':[x['geometry_type'] for x in result['links'][s][kind]]}
    result['ros2_control']=[ET.tostring(e,encoding='unicode') for e in root.findall('ros2_control')]
    result['gazebo']=[ET.tostring(e,encoding='unicode') for e in root.findall('gazebo')]
    return result


def turning():
    L,T=0.650,0.605
    def from_radius(R):
        return {'rear_axle_center_radius':R,'center_rad':math.atan(L/R),
                'inner_rad':math.atan(L/(R-T/2)),'outer_rad':math.atan(L/(R+T/2)),
                'outer_front_wheel_center_radius':math.hypot(R+T/2,L),
                'rear_outer_inner_speed_ratio':(R+T/2)/(R-T/2),
                'rear_inner_speed_at_center_1_5_m_s':1.5*(R-T/2)/R/.165,
                'rear_outer_speed_at_center_1_5_m_s':1.5*(R+T/2)/R/.165}
    out={'driver_center_0_461':from_radius(L/math.tan(.461)),
         'driver_inner_0_58':from_radius(L/math.tan(.58)+T/2),
         'manual_inner_33deg':from_radius(L/math.tan(math.radians(33))+T/2),
         'manual_R1_6_as_rear_center_hypothesis':from_radius(1.6),
         'manual_R1_6_as_outer_front_center_hypothesis':from_radius(math.sqrt(1.6**2-L**2)-T/2),
         'lcas_individual_0_461_as_inner_hypothesis':from_radius(L/math.tan(.461)+T/2),
         'gazebo_individual_0_69_as_inner_hypothesis':from_radius(L/math.tan(.69)+T/2)}
    out['manual_inner_rad']=math.radians(33)
    out['driver_inner_minus_manual_deg']=math.degrees(.58)-33
    out['required_straight_wheel_speed_rad_s']=1.5/.165
    # LCAS parallel steering lines imply different ICCs; their separation remains track.
    out['lcas_parallel_steer_0_461_icc_y']={'left':.585/2+.65142/math.tan(.461),
                                         'right':-.585/2+.65142/math.tan(.461)}
    h,w,q=.60986,.47,.461
    out['agilex_plugin_formula_probe']={'h':h,'w':w,'input_rad':q,
        'configured_left_at_index_RIGHT':math.atan2(2*h*math.tan(q),2*h+w/2*math.tan(q)),
        'configured_right_at_index_LEFT':math.atan2(2*h*math.tan(q),2*h-w/2*math.tan(q))}
    return out


def flatten(obj,path=''):
    if isinstance(obj,dict) and obj:
        result={}
        for k,v in sorted(obj.items()): result.update(flatten(v,path+'/'+k))
        return result
    if isinstance(obj,list) and obj:
        result={}
        for i,v in enumerate(obj): result.update(flatten(v,path+'/'+str(i)))
        return result
    return {path:obj}


def semantic_diff(base,lcas):
    def canonical(m,is_lcas):
        jr=m['joint_roles']; linkmap={'base_link':'root' if is_lcas else 'chassis'}
        if is_lcas: linkmap['chassis']='chassis'
        jointmap={v:k for k,v in jr.items()}
        for r,n in jr.items():linkmap[m['joints'][n]['child']]=r
        out={'links':{},'joints':{},'gazebo':m['gazebo'],'ros2_control':m['ros2_control']}
        for n,d in m['links'].items():out['links'][linkmap.get(n,n)]={'original_name':n,**d}
        for n,d in m['joints'].items():
            x={k:v for k,v in d.items() if k not in ('axis_base_zero','position_base')}
            x['parent']=linkmap.get(x['parent'],x['parent']);x['child']=linkmap.get(x['child'],x['child'])
            out['joints'][jointmap.get(n,n)]={'original_name':n,**x}
        return flatten(out)
    a,b=canonical(base,False),canonical(lcas,True); changes=[]
    def category(path):
        if 'mimic' in path:return 'E','1:1 coupling is not exact Ackermann; do not adopt'
        if '/inertial' in path:return 'E','LCAS mass/COM/inertia lacks per-link manufacturer validation'
        if '/steer_' in path and '/axis_local' in path:return 'A','Vertical steering correction supported by transforms and AgileX Gazebo'
        if '/steer_' in path and path.endswith(('/limit/lower','/limit/upper')):
            return ('E','Unlocking is an A improvement, but this exact 0.461 individual bound is unverified')
        if '/rear_' in path and '/limit/velocity' in path:return 'B','Permits official straight speed; exact 10 rad/s hardware limit is still unverified'
        if '/limit/effort' in path or '/dynamics' in path:return 'E','Simulation tuning; hardware value unverified'
        if '/steer_' in path and '/collision' in path:return 'A','Removes duplicated wheel collider'
        if '/steer_' in path and '/visual' in path:return 'A','Removes duplicated wheel visual; exact marker size is not a hardware value'
        if path.startswith('/ros2_control') or path.startswith('/gazebo'):return 'C','Gazebo/ROS2-specific integration; do not copy to Isaac'
        if 'original_name' in path:return 'D','Naming/structural organization, not a hardware correction'
        if '/axis_local' in path:return 'D','Wheel sign convention standardized; physical axle direction already transverse'
        if '/origin' in path:return 'E','Frame offset/symmetrization not established from official dimensions'
        if '/geometry' in path or '/resolved_mesh' in path or '/material' in path:return 'D','DAE/material/triangle count change; shape fidelity still needs checking'
        return 'D','Structural addition/removal; see full diff and report for adoption decision'
    for p in sorted(a.keys()|b.keys()):
        if p in a and p in b and a[p]==b[p]: continue
        cat,why=category(p)
        changes.append({'path':p,'base_present':p in a,'lcas_present':p in b,'base':a.get(p),'lcas':b.get(p),'class':cat,'reason':why})
    return changes


def detail_markdown(models):
    out=['# 자동 산출 상세표','', '단위: m, rad, kg, kg·m², N·m, rad/s. `—`는 미지정이며 0과 다르다.','']
    for label,m in models.items():
        out += [f'## {label}','',f"총 mass **{m['total_mass_kg']:.12g} kg**; links {m['counts']['links']}, joints {m['counts']['joints']}.",'',
                '| Joint | Type | Parent → Child | Origin xyz | Origin rpy | Local axis | Base axis (q=0) | Limit lower/upper/velocity/effort | Mimic |',
                '|---|---|---|---|---|---|---|---|---|']
        def fmt(v): return ' '.join(f'{x:.9g}' for x in v)
        for n,j in m['joints'].items():
            limit=j['limit'] or {}
            out += [f"| {n} | {j['type']} | {j['parent']} → {j['child']} | {fmt(j['origin']['xyz'])} | {fmt(j['origin']['rpy'])} | {fmt(j['axis_local'])} | {fmt(j['axis_base_zero'])} | {' / '.join(limit.get(k,'—') for k in ('lower','upper','velocity','effort'))} | {j['mimic'] or '—'} |"]
        out += ['', '| Link | Mass | COM xyz / rpy (link frame) | Inertia [ixx,iyy,izz; ixy,ixz,iyz] |','|---|---|---|---|']
        for n,l in m['links'].items():
            i=l['inertial']
            if i is None:out += [f'| {n} | 미지정 | — | — |'];continue
            out += [f"| {n} | {i['mass']:.12g} | {fmt(i['origin']['xyz'])} / {fmt(i['origin']['rpy'])} | {fmt([i['tensor'][k] for k in ('ixx','iyy','izz','ixy','ixz','iyz')])} |"]
        out += ['', '| Link | Visual geometry | Collision geometry |','|---|---|---|']
        for n,l in m['links'].items():
            values=['; '.join(x['geometry_type']+': '+json.dumps(x['geometry'],ensure_ascii=False) for x in l[k]) or '없음' for k in ('visual','collision')]
            out += [f'| {n} | {values[0]} | {values[1]} |']
        out += ['']
    return '\n'.join(out)+'\n'


def proposed_axis_patch(source):
    """Return a review-only diff; never write the proposed URDF."""
    # Preserve vendor CRLF in diff context; read_text() normalizes it and makes
    # an otherwise correct patch fail to match the on-disk baseline.
    before = source.read_bytes().decode('utf-8')
    after = before
    for name in ('front_steer_left_joint', 'front_steer_right_joint'):
        pattern = r'<joint\b[^>]*\bname="' + re.escape(name) + r'"[^>]*>.*?</joint>'
        matches = list(re.finditer(pattern, after, re.S))
        require(len(matches) == 1, f'Expected one steering joint: {name}')
        match = matches[0]
        joint = ET.fromstring(match.group())
        if joint.find('axis').get('xyz') == '0 1 0':
            continue  # Corrected source: no axis-only proposal remains.
        require(joint.find('axis').get('xyz') in ('0 0 1', '0 0 -1'),
                f'Steering baseline changed: {name}; review proposal again')
        block, count = re.subn(r'(<axis\s+xyz=")[^"]+("\s*/>)',
                               r'\g<1>0 1 0\2', match.group())
        require(count == 1, f'Expected one axis element: {name}')
        after = after[:match.start()] + block + after[match.end():]
    relative = source.relative_to(ROOT).as_posix()
    return ''.join(difflib.unified_diff(before.splitlines(True), after.splitlines(True),
                                      fromfile='a/' + relative, tofile='b/' + relative))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path,help='Write evidence under project docs/; default prints JSON only')
    args=parser.parse_args()
    paths={'AgileX Base':AGX/'urdf/hunter2_base.urdf','AgileX Gazebo':AGX/'urdf/hunter2_base_gazebo.xacro',
           'LCAS':LCAS/'description/robot.urdf.xacro','Current':ROOT/'assets/hunter2/source/hunter2_sim.urdf'}
    roots={k:ET.parse(p).getroot() for k,p in paths.items() if k!='LCAS'}
    reader=RestrictedXacro();roots['LCAS']=reader.read(paths['LCAS'])
    models={k:inspect_model(roots[k],p,k=='LCAS') for k,p in paths.items()}
    repos={}
    for name in ('ugv_gazebo_sim','hunter_robot','hunter_ros2_reference'):
        p=ROOT/'third_party'/name
        repos[name]={'commit':subprocess.check_output(['git','-C',str(p),'rev-parse','HEAD'],text=True).strip(),
                     'remote':subprocess.check_output(['git','-C',str(p),'remote','get-url','origin'],text=True).strip(),
                     'clean':not bool(subprocess.check_output(['git','-C',str(p),'status','--porcelain'],text=True).strip())}
    inputs=set(paths.values())|reader.files|{Path(__file__).resolve()}
    inputs.update(LCAS.glob('description/*.xacro')); inputs.update(LCAS.glob('config/*.yaml'))
    inputs.update((LCAS.parent/'hunter_gazebo/launch').glob('*.py'))
    inputs.add(LCAS.parent/'README.md')
    inputs.update((AGX.parent/'hunter2_control/config').glob('*.yaml'))
    inputs.add(AGX.parent/'steer_bot_hardware_gazebo/src/steer_bot_hardware_gazebo.cpp')
    inputs.add(AGX.parent/'steer_bot_hardware_gazebo/include/steer_bot_hardware_gazebo/steer_bot_hardware_gazebo.h')
    inputs.update((AGX.parent/'hunter2_gazebo/launch').glob('*.launch'))
    inputs.update((AGX.parent/'hunter2_control/launch').glob('*.launch'))
    driver=ROOT/'third_party/hunter_ros2_reference/hunter_base/include/hunter_base/hunter_params.hpp'
    inputs.add(driver)
    body=re.search(r'struct HunterV2Params\s*\{(.*?)\};',driver.read_text(),re.S).group(1)
    body=re.sub(r'//[^\n]*','',body)
    driver_params={k:float(v) for k,v in re.findall(r'static constexpr double\s+(\w+)\s*=\s*([\d.]+)\s*;',body)}
    require(all(driver_params[k]==v for k,v in {'track':.605,'wheelbase':.650,'wheel_radius':.165,'max_steer_angle':.58,'max_steer_angle_central':.461,'max_linear_speed':1.5}.items()),'Driver parameters changed: review numerical reference cases')
    for m in models.values():inputs.update(ROOT/p for p in m['mesh_inventory'])
    changes=semantic_diff(models['AgileX Base'],models['LCAS'])
    result={'repositories':repos,'input_sha256':{str(p.relative_to(ROOT)):sha(p) for p in sorted(inputs)},
            'xacro_method':'restricted fail-closed static expansion; no ROS/xacro installed',
            'models':models,'official_driver_parameters':driver_params,'nominal_official_geometry_turning_calculations':turning(),
            'base_to_lcas_semantic_changes':changes,'runtime_physics_validated':False}
    text=json.dumps(result,indent=2,ensure_ascii=False)+'\n'
    if args.output_dir:
        out=args.output_dir.resolve()
        require(out.is_relative_to(ROOT/'docs'),'Evidence output must be under project docs/')
        out.mkdir(parents=True,exist_ok=True)
        (out/'comparison.json').write_text(text)
        (out/'model_details.md').write_text(detail_markdown(models))
        (out/'proposed_steering_axis.patch').write_text(proposed_axis_patch(paths['Current']))
        def xml(root):
            r=copy.deepcopy(root);ET.indent(r);return ET.tostring(r,encoding='unicode')+'\n'
        (out/'lcas_expanded_for_analysis.xml').write_text(xml(roots['LCAS']))
        (out/'base_vs_lcas.diff').write_text(''.join(difflib.unified_diff(xml(roots['AgileX Base']).splitlines(True),xml(roots['LCAS']).splitlines(True),fromfile='AgileX Base',tofile='LCAS expanded')))
        change_lines=['# Semantic differences (role-mapped, derived metrics excluded)','', '| Path | Base | LCAS | Class | Reason |','|---|---|---|---|---|']
        for c in changes:
            def cell(v,present):return json.dumps(v,ensure_ascii=False).replace('|','\\|').replace('\n',' ') if present else '(absent)'
            change_lines.append(f"| `{c['path']}` | {cell(c['base'],c['base_present'])} | {cell(c['lcas'],c['lcas_present'])} | {c['class']} | {c['reason']} |")
        (out/'semantic_diff.md').write_text('\n'.join(change_lines)+'\n')
        print(json.dumps({'output_dir':str(out.relative_to(ROOT)),'semantic_changes':len(changes),
                          'masses':{k:m['total_mass_kg'] for k,m in models.items()}},ensure_ascii=False))
    else: print(text,end='')


if __name__=='__main__':
    main()
