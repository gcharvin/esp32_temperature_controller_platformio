# /// script
# requires-python = ">=3.11"
# dependencies = ["pillow>=11,<12"]
# ///
"""Route only added indicator branches; preserve all original native copper.

uv run route_diagnostics.py --workdir <temporary working directory>
The first run snapshots each board there; subsequent attempts reuse that snapshot.
Conservative raster A* is followed by native zone refill and KiCad DRC/parity.
"""
import argparse
import heapq
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
PYTHON='C:/Program Files/KiCad/10.0/bin/python.exe'
STEP=.2

def native_extract(source,out):
 import pcbnew as p
 board=p.LoadBoard(str(source));pads=[];tracks=[]
 for fp in board.GetFootprints():
  for pad in fp.Pads():
   bb=pad.GetBoundingBox();point=pad.GetPosition()
   pads.append(dict(ref=fp.GetReference(),pin=pad.GetNumber(),net=pad.GetNetname(),
    x=p.ToMM(point.x)-60,y=p.ToMM(point.y)-55,
    bounds=[p.ToMM(bb.GetLeft())-60,p.ToMM(bb.GetTop())-55,p.ToMM(bb.GetRight())-60,p.ToMM(bb.GetBottom())-55],
    layers=[i for i,l in enumerate((p.F_Cu,p.B_Cu)) if pad.IsOnLayer(l)]))
 for t in board.GetTracks():
  a=t.GetStart();b=t.GetEnd()
  row=dict(net=t.GetNetname(),a=[p.ToMM(a.x)-60,p.ToMM(a.y)-55],b=[p.ToMM(b.x)-60,p.ToMM(b.y)-55])
  if isinstance(t,p.PCB_VIA):row.update(via=True,width=p.ToMM(t.GetWidth(p.F_Cu)),layers=[0,1])
  else:row.update(via=False,width=p.ToMM(t.GetWidth()),layers=[0 if t.GetLayer()==p.F_Cu else 1])
  tracks.append(row)
 out.write_text(json.dumps(dict(pads=pads,tracks=tracks)),encoding='utf-8')

def copper_signature(board):
 import pcbnew as p
 result={}
 for t in board.GetTracks():
  isvia=isinstance(t,p.PCB_VIA)
  result[t.m_Uuid.AsString()]=(isvia,t.GetNetname(),tuple(t.GetStart()),tuple(t.GetEnd()),
                            t.GetWidth(p.F_Cu) if isvia else t.GetWidth(),t.GetLayer(),t.GetDrill() if isvia else None)
 return result

def native_apply(source,target,routes):
 import pcbnew as p
 board=p.LoadBoard(str(source));board.SetFileName(str(target))
 nets={n.GetNetname():n for n in board.GetNetsByNetcode().values()}
 def point(v):return p.VECTOR2I(p.FromMM(v[0]+60),p.FromMM(v[1]+55))
 original=copper_signature(board)
 for row in json.loads(routes.read_text(encoding='utf-8')):
  if row.get('via'):
   t=p.PCB_VIA(board);t.SetPosition(point(row['a']));t.SetWidth(p.FromMM(.8));t.SetDrill(p.FromMM(.4));t.SetLayerPair(p.F_Cu,p.B_Cu)
  else:
   t=p.PCB_TRACK(board);t.SetStart(point(row['a']));t.SetEnd(point(row['b']));t.SetWidth(p.FromMM(.25));t.SetLayer(p.F_Cu if row['layers']==[0] else p.B_Cu)
  t.SetNet(nets[row['net']]);board.Add(t)
 after=copper_signature(board)
 assert original.keys()<=after.keys() and all(after[key]==row for key,row in original.items())
 p.ZONE_FILLER(board).Fill(board.Zones());p.SaveBoard(str(target),board)
 print('Preserved',len(original),'original tracks/vias; added',len(json.loads(routes.read_text(encoding='utf-8'))))

def plan(data,spec,width,height,keepouts,skip_unconnected=False):
 from PIL import Image,ImageDraw
 nx=round(width/STEP)+1;ny=round(height/STEP)+1;size=nx*ny
 rows=data['tracks'];added=[]
 newrefs={r[k] for r in spec for k in ('led','resistor')}
 def cell(x,y):return round(x/STEP),round(y/STEP)
 def masks(net,radius):
  images=[Image.new('1',(nx,ny)) for _ in range(2)];draws=[ImageDraw.Draw(i) for i in images]
  margin=radius+.25+.09
  def bounds(x0,y0,x1,y1):return (math.floor(x0/STEP),math.floor(y0/STEP),math.ceil(x1/STEP),math.ceil(y1/STEP))
  for draw in draws:
   draw.rectangle((0,0,nx-1,ny-1),outline=1,width=math.ceil((.5+radius)/STEP))
   for x,y,w,h in keepouts:draw.rectangle(bounds(x-radius,y-radius,x+w+radius,y+h+radius),fill=1)
  for pad in data['pads']:
   if pad['net']==net:continue
   x0,y0,x1,y1=pad['bounds']
   for l in pad['layers']:draws[l].rectangle(bounds(x0-margin,y0-margin,x1+margin,y1+margin),fill=1)
  for row in rows:
   if row['net']==net:continue
   r=row['width']/2+margin;a=row['a'];b=row['b']
   for l in row['layers']:
    draw=draws[l]
    if not row['via']:draw.line([cell(*a),cell(*b)],fill=1,width=math.ceil(2*r/STEP)+1)
    for x,y in (a,b):draw.ellipse(bounds(x-r,y-r,x+r,y+r),fill=1)
  return [im.load() for im in images]
 def connect(pad,targets):
  net=pad['net'];blocked=masks(net,.125);via_blocked=masks(net,.4)
  # Target at any existing same-net pad or existing route vertex.
  targets=sorted(targets,key=lambda p:math.hypot(p['x']-pad['x'],p['y']-pad['y']))[:36]
  goalmap={}
  for target in targets:
   x,y=cell(target['x'],target['y'])
   for l in target['layers']:
    if 0<=x<nx and 0<=y<ny and not blocked[l][x,y]:goalmap[(x,y,l)]=target
  assert goalmap,(pad,'no unobstructed targets')
  goals=list(goalmap)
  def heuristic(x,y):return min(math.hypot(x-g[0],y-g[1]) for g in goals)
  queue=[];costs={};prev={};sx,sy=cell(pad['x'],pad['y'])
  for l in pad['layers']:
   state=(sx,sy,l)
   assert not blocked[l][sx,sy],(pad['ref'],pad['pin'],'source blocked',state)
   costs[state]=0;prev[state]=None;heapq.heappush(queue,(heuristic(sx,sy),0,state))
  found=None
  while queue:
   _,cost,state=heapq.heappop(queue)
   if cost!=costs.get(state):continue
   x,y,l=state
   if state in goalmap:found=state;break
   for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1),(0,0)):
    xx=x+dx;yy=y+dy;ll=l
    if dx==dy==0:
     ll=1-l
     if any(via_blocked[i][x,y] for i in (0,1)):continue
     step=35
    else:
     if not(0<=xx<nx and 0<=yy<ny) or blocked[l][xx,yy]:continue
     if dx and dy and (blocked[l][x,yy] or blocked[l][xx,y]):continue
     step=math.hypot(dx,dy)
    nxt=(xx,yy,ll);new=cost+step
    if new>=costs.get(nxt,float('inf')):continue
    costs[nxt]=new;prev[nxt]=state;heapq.heappush(queue,(new+heuristic(xx,yy),new,nxt))
  assert found is not None,(pad['ref'],pad['pin'],net,'no route')
  target=goalmap[found];path=[];state=found
  while state is not None:path.append(state);state=prev[state]
  path.reverse()
  # Keep exact pad/track coordinates at both ends; grid error is <0.15 mm.
  nodes=[(pad['x'],pad['y'],path[0][2])]+[(x*STEP,y*STEP,l) for x,y,l in path]+[(target['x'],target['y'],path[-1][2])]
  compact=[]
  for node in nodes:
   if compact and node==compact[-1]:continue
   if len(compact)>=2:
    a,b=compact[-2:]
    if a[2]==b[2]==node[2] and abs((b[0]-a[0])*(node[1]-b[1])-(b[1]-a[1])*(node[0]-b[0]))<1e-8:compact.pop()
   compact.append(node)
  for a,b in zip(compact,compact[1:]):
   if a[2]!=b[2]:row=dict(net=net,a=list(a[:2]),b=list(a[:2]),via=True,width=.8,layers=[0,1])
   elif math.dist(a[:2],b[:2])>1e-7:row=dict(net=net,a=list(a[:2]),b=list(b[:2]),via=False,width=.25,layers=[a[2]])
   else:continue
   rows.append(row);added.append(row)
  print(pad['ref']+'.'+pad['pin'],net,'->',target.get('ref','track'),target.get('pin',''),flush=True)
 # Each resistor/LED pair is routed before its supply and return branches.
 newpads=[pad for pad in data['pads'] if pad['ref'] in newrefs and not
          (skip_unconnected and pad['net'].removeprefix('/').startswith('unconnected-'))]
 connected=set()
 for pad in sorted(newpads,key=lambda p:(0 if '/LED_' in p['net'] else 1,p['net'],p['ref'],p['pin'])):
  key=pad['ref'],pad['pin']
  if key in connected:continue
  targets=[t for t in data['pads'] if t['net']==pad['net'] and (t['ref'] not in newrefs or (t['ref'],t['pin']) in connected or '/LED_' in pad['net']) and (t['ref'],t['pin'])!=key]
  for row in rows:
   if row['net']==pad['net']:
    for x,y in (row['a'],row['b']):targets.append(dict(x=x,y=y,layers=row['layers']))
  assert targets,(key,pad['net'])
  connect(pad,targets);connected.add(key)
  if '/LED_' in pad['net']:connected.update((t['ref'],t['pin']) for t in targets if 'ref' in t)
 return added

def main():
 parser=argparse.ArgumentParser();parser.add_argument('--extract',nargs=2);parser.add_argument('--apply',nargs=3);parser.add_argument('--workdir',type=Path);parser.add_argument('--board');args=parser.parse_args()
 if args.extract:native_extract(*map(Path,args.extract));return
 if args.apply:native_apply(*map(Path,args.apply));return
 assert args.workdir;args.workdir.mkdir(exist_ok=True,parents=True)
 layout=json.loads((ROOT/'placement.json').read_text(encoding='utf-8'))
 specs=json.loads((ROOT/'diagnostic_leds.json').read_text(encoding='utf-8'))['boards']
 for board in layout['boards']:
  name=board['name']
  if args.board and args.board!=name:continue
  source=ROOT/name/(name+'.kicad_pcb');base=args.workdir/(name+'_base.kicad_pcb')
  if not base.exists():shutil.copy2(source,base)
  datafile=args.workdir/(name+'_copper.json');routes=args.workdir/(name+'_led_routes.json')
  subprocess.run([PYTHON,__file__,'--extract',str(base),str(datafile)],check=True)
  keepouts=board['clip_bands_mm']+board['side_support_bands_mm']+board['clip_side_contact_patches_mm']
  if name=='pcb_a_display':keepouts+=[[0,17,12.5,46]]
  result=plan(json.loads(datafile.read_text()),specs[name],*board['board_mm'][:2],keepouts)
  routes.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
  subprocess.run([PYTHON,__file__,'--apply',str(base),str(source),str(routes)],check=True)

if __name__=='__main__':main()
